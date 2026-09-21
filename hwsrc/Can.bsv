package Can;

// CAN 2.0B 控制器（Bosch CAN Specification 2.0 Part B）：一个发送缓冲、一个接收缓冲、一组验收滤波。
// 帧的位置表在 CanFrame、故障界定在 CanFault（都是 BH），CRC-15 用 hwcore 的 Gf2。
// 这里分两层：每拍推进时间份额与同步（第 10 节），每个采样点推进一次 MAC。
// 全部状态只归 step 一条规则写；命令脉冲在总线方法之后才有，由 CReg 递到下一拍。

import Vector::*;
import RegIf::*;
import CanRegs::*;
import CanFrame::*;
import CanFault::*;
import Gf2::*;

typedef struct {
  Bool filter;
} CanCfg;

interface CanPins;
  (* always_ready, result = "can_tx" *) method Bit#(1) can_tx;
  (* always_ready, always_enabled, prefix = "" *)
  method Action can_rx((* port = "can_rx" *) Bit#(1) v);
endinterface

interface CanIfc#(numeric type aw, numeric type dw);
  interface RegIf#(aw, dw) regs;
  interface CanPins        pins;
  (* always_ready *) method Bool irq;
endinterface

typedef enum {
  Off, Integrate, Idle, Body, CrcDel, AckSlot, AckDel, Eof, Inter, Suspend, Flag, PFlag, Delim, Quiet
} St deriving (Bits, Eq);

// 末次错误码，照 Bosch C_CAN 的编码
UInt#(4) lecStuff = 1;
UInt#(4) lecForm  = 2;
UInt#(4) lecAck   = 3;
UInt#(4) lecBit1  = 4;   // 发隐性见显性
UInt#(4) lecBit0  = 5;   // 发显性见隐性
UInt#(4) lecCrc   = 6;

module mkCan#(CanCfg cfg)(CanIfc#(aw, dw))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 1, dw),
              Add#(_c, 2, dw), Add#(_d, 3, dw), Add#(_e, 4, dw), Add#(_f, 8, dw),
              Add#(_g, 9, dw), Add#(_h, 29, dw), Add#(_i, 32, dw));

  CanRegsIfc#(aw, dw) r <- mkCanRegs(CanRegsCfg { filter: cfg.filter });

  Wire#(Bit#(1)) rxPin <- mkBypassWire;
  // 收进来的线对本时钟是异步的，打两拍；边沿与采样都看打过两拍的值
  Reg#(Bit#(1)) rx1    <- mkReg(1);
  Reg#(Bit#(1)) rx2    <- mkReg(1);
  Reg#(Bit#(1)) rxPrev <- mkReg(1);
  Reg#(Bit#(1)) txR    <- mkReg(1);

  // ---- 位定时 ----
  Reg#(UInt#(8)) presc  <- mkReg(0);
  Reg#(UInt#(5)) q      <- mkReg(0);       // 这一位里的第几个时间份额，0 是同步段
  Reg#(UInt#(3)) ext1   <- mkReg(0);       // 正相位误差把 PHASE_SEG1 拉长了几份
  Reg#(UInt#(3)) cut2   <- mkReg(0);       // 负相位误差把 PHASE_SEG2 缩短了几份
  Reg#(Bool)     synced <- mkReg(False);   // 同步规则 1：一位只同步一次
  Reg#(Bit#(1))  lastS  <- mkReg(1);       // 上一个采样点的值（同步规则 2）
  Reg#(Bit#(1))  nextTx <- mkReg(1);       // 采样点上定好，下一位开头放出去

  // ---- MAC ----
  Reg#(St)        st         <- mkReg(Off);
  Reg#(Bool)      isTx       <- mkReg(False);
  Reg#(UInt#(7))  pos        <- mkReg(0);     // 解开填充后的第几位
  Reg#(UInt#(7))  nb         <- mkReg(127);   // CRC 从第几位开始；收到 DLC 之前填一个到不了的数
  Reg#(Bool)      extR       <- mkReg(False);
  Reg#(UInt#(3))  run        <- mkReg(0);
  Reg#(Bit#(1))   lastB      <- mkReg(1);
  Reg#(Bit#(15))  crc        <- mkReg(0);
  Reg#(Bit#(103)) sh         <- mkReg(0);
  Reg#(Bool)      crcBad     <- mkReg(False);
  Reg#(UInt#(4))  cnt        <- mkReg(0);
  Reg#(Bool)      recSeen    <- mkReg(False); // 定界符等到了第一个隐性
  Reg#(Bool)      afterErr   <- mkReg(False); // 定界符前面是错误标志而不是过载标志（规则 2 只认前者）
  Reg#(Bool)      firstAfter <- mkReg(False);
  Reg#(UInt#(8))  domRun     <- mkReg(0);     // 标志之后连着的显性位
  Reg#(Bool)      pDom       <- mkReg(False); // 发消极错误标志时见过显性
  Reg#(Bool)      deferT     <- mkReg(False); // 规则 3 例外 1 待定
  Reg#(UInt#(3))  eqRun      <- mkReg(0);
  Reg#(Counts)    cnts       <- mkReg(Counts { tec: 0, rec: 0 });
  Reg#(UInt#(4))  lec        <- mkReg(0);
  Reg#(UInt#(4))  quiet      <- mkReg(0);     // 连续隐性位数（并入总线、总线关闭恢复）
  Reg#(UInt#(8))  boSeq      <- mkReg(0);
  Reg#(Frame)     rxTmp      <- mkReg(unpack(0));
  Reg#(Frame)     rxBuf      <- mkReg(unpack(0));
  Reg#(Frame)     txF        <- mkReg(unpack(0));
  Reg#(Bool)      txPend     <- mkReg(False);
  Reg#(Bool)      abortR     <- mkReg(False);

  Reg#(Bool) reqTx[2] <- mkCReg(2, False);
  Reg#(Bool) reqAb[2] <- mkCReg(2, False);

  // 写 cmd 时两个字段的 swmod 一起跳，要看写进来的是不是 1，否则写 tx 会顺带撤销
  rule mark;
    if (r.cmd_tx_wr && r.cmd_tx_wr_val == 1) reqTx[1] <= True;
    if (r.cmd_abort_wr && r.cmd_abort_wr_val == 1) reqAb[1] <= True;
  endrule

  UInt#(8) brp   = unpack(r.btr_brp);
  UInt#(5) ts1v  = unpack(zeroExtend(r.btr_ts1)) + 1;
  UInt#(5) tseg1 = ts1v < 2 ? 2 : ts1v;
  UInt#(5) tseg2 = unpack(zeroExtend(r.btr_ts2)) + 1;
  UInt#(5) sjw0  = unpack(zeroExtend(r.btr_sjw)) + 1;
  // PROP_SEG 与 PHASE_SEG1 合在 ts1 里，跳转宽度按 PHASE_SEG2 截，缩短时不越过采样点
  UInt#(5) sjw   = sjw0 > tseg2 ? tseg2 : sjw0;

  Vector#(8, Bit#(8)) txBytes = unpack({r.txd1, r.txd0});
  Frame txReq = Frame { ext: r.txid_ext == 1, rtr: r.txid_rtr == 1, ident: r.txid_id,
                        dlc: r.txdlc, payload: txBytes };

  function Bool accept(Frame f);
    Bool idOk  = ((f.ident ^ r.fid_id) & r.fmask_id) == 0;
    Bool extOk = r.fmask_ext == 0 || pack(f.ext) == r.fid_ext;
    return !cfg.filter || (idOk && extOk);
  endfunction

  rule step;
    Bool    en   = r.ctrl_en == 1;
    Bit#(1) b    = rx2;
    Bool    fall = rxPrev == 1 && b == 0;
    Mode    m0   = mode(cnts);

    // ================= 位定时 =================
    Bool sendingDom = isTx && txR == 0;
    Bool hardOk     = st == Idle || st == Suspend || st == Integrate || st == Quiet;
    UInt#(5) sp0    = tseg1 + zeroExtend(ext1);
    UInt#(5) bend0  = 1 + tseg1 + zeroExtend(ext1) + tseg2 - zeroExtend(cut2);

    Bool     restart  = False;   // 边沿这一拍当作一位的同步段
    Bool     endEarly = False;   // 边沿在采样点之后：上一位就此结束，放出下一位
    UInt#(3) nExt1    = ext1;
    UInt#(3) nCut2    = cut2;
    Bool     nSynced  = synced;
    // 只用隐性到显性的边沿（同步规则 4 的前提）
    if (en && fall && !synced && lastS == 1) begin
      if (hardOk && !sendingDom) begin
        restart = True; endEarly = q > sp0;
      end else if (q == 0) begin
        noAction;
      end else if (q <= sp0) begin
        // 同步规则 4：自己在发显性位时不跟正相位误差的边沿
        if (!sendingDom) begin
          if (q <= sjw) restart = True;
          else begin nExt1 = truncate(sjw); nSynced = True; end
        end
      end else begin
        UInt#(5) e = bend0 - q;
        if (e <= sjw) begin restart = True; endEarly = True; end
        else begin nCut2 = truncate(sjw); nSynced = True; end
      end
    end

    UInt#(8) nPresc   = presc + 1;
    UInt#(5) nq       = q;
    Bool     newBit   = False;
    Bool     doSample = False;
    if (!en) begin
      nPresc = 0; nq = 0;
    end else if (restart) begin
      nPresc = 1; nq = 0;
      if (brp == 0) begin nPresc = 0; nq = 1; end
      nExt1 = 0; nCut2 = 0; nSynced = True;
      newBit = endEarly;
    end else if (presc >= brp) begin
      UInt#(5) sp   = tseg1 + zeroExtend(nExt1);
      UInt#(5) bend = 1 + tseg1 + zeroExtend(nExt1) + tseg2 - zeroExtend(nCut2);
      nPresc = 0;
      doSample = q == sp;
      // 判「不小于」：运行中把 ts2 改小，份额计数已经越过新的位尾也不许卡住
      if (q >= bend - 1) begin nq = 0; newBit = True; nExt1 = 0; nCut2 = 0; nSynced = False; end
      else nq = q + 1;
    end

    // ================= MAC =================
    St        nst         = st;
    Bool      nIsTx       = isTx;
    UInt#(7)  nPos        = pos;
    UInt#(7)  nNb         = nb;
    Bool      nExtR       = extR;
    UInt#(3)  nRun        = run;
    Bit#(1)   nLastB      = lastB;
    Bit#(15)  nCrc        = crc;
    Bit#(103) nSh         = sh;
    Bool      nCrcBad     = crcBad;
    UInt#(4)  nCnt        = cnt;
    Bool      nRecSeen    = recSeen;
    Bool      nAfterErr   = afterErr;
    Bool      nFirstAfter = firstAfter;
    UInt#(8)  nDomRun     = domRun;
    Bool      nPDom       = pDom;
    Bool      nDeferT     = deferT;
    UInt#(3)  nEqRun      = eqRun;
    UInt#(4)  nLec        = lec;
    UInt#(4)  nQuiet      = quiet;
    UInt#(8)  nBoSeq      = boSeq;
    Frame     nRxTmp      = rxTmp;
    Bool      nTxPend     = txPend;
    Bool      nAbort      = abortR;
    Bit#(1)   ntx         = nextTx;
    Bit#(1)   nLastS      = lastS;

    Maybe#(Event) ev = tagged Invalid;
    Bool     errNow    = False;   // 查出错误：下一位起发错误标志
    UInt#(4) errCode   = 0;
    Bool     errCount  = True;    // 规则 3 的两个例外不计数
    Bool     deferNow  = False;
    Bool     flagBit   = False;   // 发主动错误标志或过载标志时的位错误
    Bool     deliver   = False;
    Bool     evTxOk    = False;
    Bool     evArb     = False;
    Bool     startSof  = False;
    Bool     startAsTx = False;
    Bool     enterIdle = False;

    Bool      txGo = txPend && !abortR && m0 != BusOff;
    Bit#(103) tb   = txBits(txF);

    if (!en) begin
      nst = Off; ntx = 1; nIsTx = False;
    end else if (st == Off) begin
      nst = Integrate; nQuiet = 0; ntx = 1;
    end else if (doSample) begin
      Bit#(1) s = b;
      Bit#(1) t = txR;
      nLastS = s;
      case (st)
        // 使能之后先见 11 个连续隐性才并入总线
        Integrate: begin
          ntx = 1;
          if (s == 0) nQuiet = 0;
          else if (quiet >= 10) begin nQuiet = 0; enterIdle = True; end
          else nQuiet = quiet + 1;
        end

        Idle: begin
          if (isTx && t == 0 && s == 1) begin errNow = True; errCode = lecBit0; end
          else if (s == 0) begin startSof = True; startAsTx = isTx && t == 0; end
          else enterIdle = True;
        end

        // ---- 帧起始到 CRC 序列：带填充 ----
        Body: begin
          Bool arb = pos <= (txF.ext ? 32 : 12);
          // CRC 场之后那一位的位置。收到 DLC 之前 nb 是 127，加 15 要在 8 位里做，7 位会回绕到 14
          UInt#(8) crcEnd = zeroExtend(nb) + 15;
          if (run == 5) begin
            // 这一位是填充位，应当与前 5 位相反
            if (isTx && s != t) begin
              errNow = True;
              if (t == 1 && arb) begin errCode = lecStuff; errCount = False; end   // 规则 3 例外 2
              else errCode = t == 1 ? lecBit1 : lecBit0;
            end else if (!isTx && s == lastB) begin
              errNow = True; errCode = lecStuff;
            end else begin
              nRun = 1; nLastB = s;
              if (zeroExtend(pos) == crcEnd) begin nst = CrcDel; ntx = 1; end
              else if (!isTx) ntx = 1;
              else ntx = pos < nb ? tb[102 - pos] : crc[14];
            end
          end else begin
            Bool lost = isTx && s != t && t == 1 && arb;
            if (isTx && s != t && !lost) begin
              errNow = True; errCode = t == 1 ? lecBit1 : lecBit0;
            end else begin
              Bool tx2 = isTx && !lost;
              if (lost) begin nIsTx = False; evArb = True; end
              Bit#(15)  c2  = crcBit(crc15Can, crc, s);
              Bit#(103) sh2 = {sh[101:0], s};
              UInt#(3)  r2  = s == lastB ? run + 1 : 1;
              UInt#(7)  p2  = pos + 1;
              Bool      e2  = pos == 13 ? s == 1 : extR;
              UInt#(7)  nb2 = pos == hdrLen(e2) - 1 ? crcStart(e2, sh2[6] == 1, sh2[3:0]) : nb;
              UInt#(8)  end2 = zeroExtend(nb2) + 15;
              if (p2 == nb2) nRxTmp = parse(nb2, sh2);
              // 15 位 CRC 也喂进同一个寄存器，收完为 0 才对
              if (zeroExtend(p2) == end2) nCrcBad = c2 != 0;
              nCrc = c2; nSh = sh2; nRun = r2; nLastB = s; nPos = p2; nExtR = e2; nNb = nb2;
              if (zeroExtend(p2) == end2 && r2 != 5) begin nst = CrcDel; ntx = 1; end
              else if (!tx2) ntx = 1;
              else if (r2 == 5) ntx = ~s;
              else if (p2 < nb2) ntx = tb[102 - p2];
              else ntx = c2[14];
            end
          end
        end

        // ---- 定长的尾巴 ----
        CrcDel: begin
          if (s == 0) begin errNow = True; errCode = isTx ? lecBit1 : lecForm; end
          else begin nst = AckSlot; ntx = (!isTx && !crcBad) ? 0 : 1; end
        end
        AckSlot: begin
          if (isTx) begin
            if (s == 1) begin
              errNow = True; errCode = lecAck;
              // 例外 1 要等消极错误标志发完才知道成不成立
              if (m0 == Passive) begin errCount = False; deferNow = True; end
            end else begin nst = AckDel; ntx = 1; end
          end else if (t == 0 && s == 1) begin
            errNow = True; errCode = lecBit0;
          end else begin
            if (t == 0) ev = tagged Valid RxOk;
            nst = AckDel; ntx = 1;
          end
        end
        AckDel: begin
          if (s == 0) begin errNow = True; errCode = isTx ? lecBit1 : lecForm; end
          else if (!isTx && crcBad) begin errNow = True; errCode = lecCrc; end   // 7.2：CRC 错在这之后才报
          else begin nst = Eof; nCnt = 0; ntx = 1; end
        end
        Eof: begin
          // 接收方不看帧结束的末位（第 5 节）
          if (s == 0 && (isTx || cnt < 6)) begin
            errNow = True; errCode = isTx ? lecBit1 : lecForm;
          end else begin
            ntx = 1;
            if (!isTx && cnt == 5) deliver = True;
            if (cnt >= 6) begin
              if (isTx) begin ev = tagged Valid TxOk; evTxOk = True; nTxPend = False; nAbort = False; end
              nst = Inter; nCnt = 0;
            end else nCnt = cnt + 1;
          end
        end

        // ---- 帧间 ----
        Inter: begin
          if (s == 0) begin
            if (cnt < 2) begin nst = Flag; nAfterErr = False; nCnt = 0; ntx = 0; end   // 过载条件 2
            else begin
              // 间歇第 3 位的显性当帧起始；有待发帧就从 ID-28 发起，刚发过帧的消极节点除外
              startSof = True; startAsTx = txGo && !(isTx && m0 == Passive);
            end
          end else if (cnt >= 2) begin
            if (isTx && m0 == Passive) begin nst = Suspend; nCnt = 0; ntx = 1; nIsTx = False; end
            else enterIdle = True;
          end else nCnt = cnt + 1;
        end
        Suspend: begin
          if (s == 0) begin startSof = True; startAsTx = False; end
          else if (cnt >= 7) enterIdle = True;
          else nCnt = cnt + 1;
        end

        // ---- 错误帧与过载帧 ----
        Flag: begin
          if (s == 1) begin
            // 规则 4、5：加 8，重发一个错误标志
            ev = tagged Valid (isTx ? TxFlagBitError : RxFlagBitError);
            nLec = lecBit0; flagBit = True;
            nCnt = 0; nAfterErr = True;
            if (m0 == Active) begin nst = Flag; ntx = 0; end
            else begin nst = PFlag; ntx = 1; nEqRun = 0; nPDom = False; end
          end else if (cnt >= 5) begin
            nst = Delim; nRecSeen = False; nFirstAfter = True; nDomRun = 0; nCnt = 0; ntx = 1;
          end else begin nCnt = cnt + 1; ntx = 0; end
        end
        PFlag: begin
          // 从标志开头起等 6 个同极性的位
          UInt#(3) er  = (cnt == 0 || s != lastS) ? 1 : eqRun + 1;
          Bool     dom = pDom || s == 0;
          ntx = 1; nEqRun = er; nPDom = dom;
          if (cnt < 15) nCnt = cnt + 1;
          if (er >= 6) begin
            nst = Delim; nRecSeen = False; nFirstAfter = True; nDomRun = 0; nCnt = 0;
            if (deferT) begin
              if (dom) ev = tagged Valid TxError;
              nDeferT = False;
            end
          end
        end
        Delim: begin
          ntx = 1;
          if (!recSeen) begin
            if (s == 0) begin
              UInt#(8) d = domRun + 1;
              nDomRun = d;
              if (firstAfter && afterErr && !isTx) ev = tagged Valid RxDomAfterFlag;   // 规则 2
              else if (d % 8 == 0) ev = tagged Valid (isTx ? TxDomRun : RxDomRun);     // 规则 6
            end else begin nRecSeen = True; nCnt = 1; end
            nFirstAfter = False;
          end else if (s == 0) begin
            if (cnt >= 7) begin nst = Flag; nAfterErr = False; nCnt = 0; ntx = 0; end  // 过载条件 3
            else begin errNow = True; errCode = lecForm; end
          end else if (cnt >= 7) begin
            nst = Inter; nCnt = 0;
          end else nCnt = cnt + 1;
        end

        // ---- 总线关闭：数满 128 次 11 个连续隐性（规则 12）----
        Quiet: begin
          ntx = 1;
          if (s == 0) nQuiet = 0;
          else if (quiet >= 10) begin
            nQuiet = 0;
            if (boSeq >= 127) begin ev = tagged Valid Recovered; nBoSeq = 0; enterIdle = True; end
            else nBoSeq = boSeq + 1;
          end else nQuiet = quiet + 1;
        end

        default: noAction;
      endcase

      if (startSof) begin
        nst = Body; nPos = 1; nRun = 1; nLastB = 0; nCrc = 0; nSh = 0;
        nNb = 127; nExtR = False; nCrcBad = False;
        nIsTx = startAsTx;
        ntx = startAsTx ? tb[101] : 1;
      end
      if (enterIdle) begin
        nst = Idle; nIsTx = txGo; ntx = txGo ? 0 : 1;
      end
      if (errNow) begin
        nLec = errCode;
        if (errCount) ev = tagged Valid (isTx ? TxError : RxError);
        nDeferT = deferNow;
        nAfterErr = True; nCnt = 0; nEqRun = 0; nPDom = False;
        // 规则 9 后半句：发哪种标志按查出错误之前的状态定
        if (m0 == Active) begin nst = Flag; ntx = 0; end
        else begin nst = PFlag; ntx = 1; end
        if (isTx && abortR) begin nTxPend = False; nAbort = False; end
      end
      if (evArb && abortR) begin nTxPend = False; nAbort = False; end
    end

    Counts nc = cnts;
    if (ev matches tagged Valid .e) nc = bump(e, cnts);
    Mode m1 = mode(nc);
    Bool becameOff = m1 == BusOff && m0 != BusOff;
    if (becameOff) begin
      // 总线关闭时撤掉待发帧，要不要重发交给软件
      nst = Quiet; ntx = 1; nTxPend = False; nAbort = False; nIsTx = False;
      nQuiet = 0; nBoSeq = 0; nDeferT = False;
    end

    // 已经有帧待发时再写 tx 不算
    if (reqTx[0] && !nTxPend && m1 != BusOff) begin txF <= txReq; nTxPend = True; nAbort = False; end
    if (reqAb[0]) begin
      if (nIsTx) nAbort = True;
      else nTxPend = False;
    end

    reqTx[0] <= False;
    reqAb[0] <= False;
    rx1 <= rxPin; rx2 <= rx1; rxPrev <= rx2;
    presc <= nPresc; q <= nq; ext1 <= nExt1; cut2 <= nCut2; synced <= nSynced;
    lastS <= nLastS; nextTx <= ntx;
    txR <= (becameOff || !en) ? 1 : (newBit ? nextTx : txR);

    st <= nst; isTx <= nIsTx; pos <= nPos; nb <= nNb; extR <= nExtR; run <= nRun; lastB <= nLastB;
    crc <= nCrc; sh <= nSh; crcBad <= nCrcBad; cnt <= nCnt; recSeen <= nRecSeen;
    afterErr <= nAfterErr; firstAfter <= nFirstAfter; domRun <= nDomRun; pDom <= nPDom;
    deferT <= nDeferT; eqRun <= nEqRun; cnts <= nc; lec <= nLec; quiet <= nQuiet; boSeq <= nBoSeq;
    rxTmp <= nRxTmp; txPend <= nTxPend; abortR <= nAbort;

    if (deliver && accept(rxTmp)) begin
      rxBuf <= rxTmp;
      r.events_rxok_set(1);
      if (r.events_rxok == 1) r.events_rxover_set(1);
    end
    if (evTxOk) r.events_txok_set(1);
    if (errNow || flagBit) r.events_err_set(1);
    if (evArb) r.events_arblost_set(1);
    if (m1 == Passive && m0 == Active) r.events_passive_set(1);
    if (becameOff) r.events_busoff_set(1);
  endrule

  rule show;
    Mode m = mode(cnts);
    r.status_state_in(m == Active ? 0 : (m == Passive ? 1 : 2));
    r.status_txpend_in(txPend ? 1 : 0);
    r.status_idle_in(st == Idle ? 1 : 0);
    r.status_lec_in(pack(lec));
    r.errcnt_tec_in(pack(cnts.tec));
    r.errcnt_rec_in(pack(cnts.rec));
    r.rxid_id_in(rxBuf.ident);
    r.rxid_ext_in(pack(rxBuf.ext));
    r.rxid_rtr_in(pack(rxBuf.rtr));
    r.rxdlc_in(rxBuf.dlc);
    Bit#(64) d = pack(rxBuf.payload);
    r.rxd0_in(d[31:0]);
    r.rxd1_in(d[63:32]);
  endrule

  interface regs = r.regs;
  interface CanPins pins;
    method Bit#(1) can_tx = txR;
    method Action can_rx(Bit#(1) v); rxPin._write(v); endmethod
  endinterface
  method Bool irq = r.ctrl_ien == 1 &&
                    (r.events_txok == 1 || r.events_rxok == 1 || r.events_err == 1 || r.events_passive == 1 ||
                     r.events_busoff == 1 || r.events_arblost == 1 || r.events_rxover == 1);
endmodule

endpackage
