"""can 的行为测试台：两个被测节点、一个脚本节点接在线与总线上，外加一个能把某一位拉成显性的注入端。

帧的位流在这里独立算：CRC-15 照 Bosch CAN 2.0 Part B 3.2.1 的伪代码写，先对 RevEng 目录的 0x059e 自证；
填充照第 6 节（填充位计入之后的连续位数，CRC 最后 5 位相同照样补一位）；定界、应答、帧结束照 3.2.1 拼。
一位 40 拍：brp 3（4 拍一个时间份额）、ts1 6（7 份）、ts2 1（2 份）、sjw 1（2 份），采样点在第 32 拍。
脚本节点与监视都按这个位宽逐拍走，不借被测件的时序。判据见 notes/规范对照/can.md。

认矩阵：`filter` 关着时不写滤波寄存器，被拒的那一帧改成照收。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
filt = bool(cfg.get("knobs", {}).get("filter", True))

BRP, TS1, TS2, SJW = 3, 6, 1, 1
BT = (BRP + 1) * (1 + (TS1 + 1) + (TS2 + 1))
if BT != 40:
    raise SystemExit("位宽不是 40 拍，测试台里按 40 拍写的常数要跟着改")
BTR = BRP | (TS1 << 8) | (TS2 << 12) | (SJW << 16)

CTRL, BTRA, STATUS, ERRCNT, TXID, TXDLC, TXD0, TXD1, CMD = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20
FID, FMASK, EVENTS = 0x34, 0x38, 0x3C
EV = {"txok": 0, "rxok": 1, "err": 2, "passive": 3, "busoff": 4, "arblost": 5, "rxover": 6}


def bits(v, n):
    return [(v >> (n - 1 - i)) & 1 for i in range(n)]


def crc15(bs):
    c = 0
    for b in bs:
        nxt = b ^ ((c >> 14) & 1)
        c = (c << 1) & 0x7FFF
        if nxt:
            c ^= 0x4599
    return c


if crc15([x for byte in b"123456789" for x in bits(byte, 8)]) != 0x059E:
    raise SystemExit("CRC-15 参考实现对不上 RevEng 目录的 0x059e，不生成测试台")


def stuff(bs):
    """填充后的位流与每一位的来历：非负是解开填充后的位置，-(p+1) 是跟在位置 p 后面的填充位。"""
    got, kind, run, last = [], [], 0, None
    for p, x in enumerate(bs):
        got.append(x)
        kind.append(p)
        run = run + 1 if x == last else 1
        last = x
        if run == 5:
            got.append(1 - x)
            kind.append(-(p + 1))
            run, last = 1, 1 - x
    return got, kind


def frame(ext, rtr, ident, dlc, data=(), crc_flip=0):
    data = (list(data) + [0] * 8)[:8]
    d = [0] + bits(ident >> 18, 11)
    d += ([1, 1] + bits(ident & 0x3FFFF, 18) + [rtr, 0, 0]) if ext else [rtr, 0, 0]
    d += bits(dlc, 4)
    n = 0 if rtr else min(dlc, 8)
    for x in data[:n]:
        d += bits(x, 8)
    body, kind = stuff(d + bits(crc15(d) ^ crc_flip, 15))
    return {"ext": ext, "rtr": rtr, "ident": ident, "dlc": dlc, "data": data, "n": n,
            "body": body, "kind": kind, "hdr": 39 if ext else 19}


def tail(ack, crcdel=1):
    return [crcdel, ack, 1] + [1] * 7


def pack4(bs):
    return sum(b << (8 * i) for i, b in enumerate(bs))


def txregs(f):
    return (f["ident"] | (f["ext"] << 29) | (f["rtr"] << 30), f["dlc"], pack4(f["data"][:4]), pack4(f["data"][4:]))


def rxregs(f):
    data = [f["data"][i] if i < f["n"] else 0 for i in range(8)]
    ident = f["ident"] if f["ext"] else (f["ident"] >> 18) << 18
    return (ident | (f["ext"] << 29) | (f["rtr"] << 30), f["dlc"], pack4(data[:4]), pack4(data[4:]))


def first(f, pred):
    for i, (b, k) in enumerate(zip(f["body"], f["kind"])):
        if pred(b, k, f["hdr"]):
            return i
    raise SystemExit("帧里找不到要改的那一位，换一帧")


scripts = []
S = []
# 连着的几条核对并进一个 action：一条核对一个状态会让 StmtFSM 展开步数超过 bsc 的上限（G0024）
pending = []


def flush():
    if pending:
        S.append("    action")
        S.append("      Bool wrong = False;")
        S.extend(pending)
        S.append("      if (wrong) bad <= True;")
        S.append("    endaction")
        pending.clear()


def emit(s):
    flush()
    S.append("    " + s)


def check(cond, fmt, *args):
    tail_ = "".join(", " + a for a in args)
    pending.append(f'      if (!({cond})) begin $display("FAIL {fmt}"{tail_}); wrong = True; end')


def eq(got, want, what):
    check(f"Bit#(32)'(zeroExtend({got})) == {want}", f"{what}: got %0h, want {want}", got)


def rng(got, lo, hi, what):
    check(f"{got} >= {lo} && {got} <= {hi}", f"{what}: got %0d, want {lo} to {hi}", got)


def lec(n):
    return f"sn[{n}][0][7:4]"


def state(n):
    return f"sn[{n}][0][1:0]"


def txpend(n):
    return f"sn[{n}][0][2:2]"


def tec(n):
    return f"sn[{n}][1][8:0]"


def rec(n):
    return f"sn[{n}][1][23:16]"


def ev(n, name):
    b = EV[name]
    return f"sn[{n}][6][{b}:{b}]"


def seen(lo, hi, want, what):
    val = ((1 << (hi - lo + 1)) - 1) if want else 0
    eq(f"seen[{hi}:{lo}]", val, what)


def rx_is(n, f, what):
    rid, dlc, d0, d1 = rxregs(f)
    eq(f"sn[{n}][2]", f"32'h{rid:08X}", f"{what}: rxid")
    eq(f"sn[{n}][3]", dlc, f"{what}: rxdlc")
    eq(f"sn[{n}][4]", f"32'h{d0:08X}", f"{what}: rxd0")
    eq(f"sn[{n}][5]", f"32'h{d1:08X}", f"{what}: rxd1")


def load(n, f):
    tid, dlc, d0, d1 = txregs(f)
    emit(f"wr({n}, 8'h{TXID:02X}, 32'h{tid:08X});")
    emit(f"wr({n}, 8'h{TXDLC:02X}, {dlc});")
    emit(f"wr({n}, 8'h{TXD0:02X}, 32'h{d0:08X});")
    emit(f"wr({n}, 8'h{TXD1:02X}, 32'h{d1:08X});")


def scripted(name, stream, jitter=False):
    scripts.append(stream)
    emit(f"// ---- {name} ----")
    emit(f"wr(0, 8'h{EVENTS:02X}, 32'h7F);")
    emit(f"action scrCase <= {len(scripts) - 1}; jitter <= {'True' if jitter else 'False'}; scrGo <= scrGo + 1; endaction")
    emit("delay(4);")
    emit("await(!scrOn);")
    emit("delay(1200);")
    emit(f'waitIdle(0, "{name}");')
    emit("snap(0);")


# ================= 上电：两个节点同一套位定时，先只开 0 号 =================
emit(f"wr(0, 8'h{BTRA:02X}, 32'h{BTR:08X});")
emit(f"wr(1, 8'h{BTRA:02X}, 32'h{BTR:08X});")
emit(f"wr(0, 8'h{CTRL:02X}, 1);")
emit("delay(600);")
emit('waitIdle(0, "after enabling node 0");')

# ================= 脚本节点发，0 号收 =================
std = frame(0, 0, 0x123 << 18, 2, [0xA5, 0x0F])
ext = frame(1, 0, 0x1ABCDE5, 8, [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF])
rem = frame(0, 1, 0x3C5 << 18, 3)

for name, f in (("a standard data frame", std), ("an extended data frame", ext), ("a standard remote frame", rem)):
    ack = len(f["body"]) + 1
    scripted(name, f["body"] + tail(1))
    seen(ack, ack, 0, f"{name}: node 0 did not acknowledge it")
    seen(ack + 2, ack + 8, 1, f"{name}: a dominant bit inside end of frame")
    rx_is(0, f, name)
    eq(ev(0, "rxok"), 1, f"{name}: rxok")
    eq(ev(0, "err"), 0, f"{name}: err")
    eq(rec(0), 0, f"{name}: REC")

# CRC 改一位：不应答，标志从 ACK 定界符的下一位起（7.2）
bad = frame(0, 0, 0x123 << 18, 2, [0xA5, 0x0F], crc_flip=1)
ack = len(bad["body"]) + 1
scripted("a corrupt CRC", bad["body"] + tail(1))
seen(ack, ack, 1, "a corrupt CRC: node 0 acknowledged it")
seen(ack + 2, ack + 7, 0, "a corrupt CRC: no error flag from the bit after the ACK delimiter")
eq(lec(0), 6, "a corrupt CRC: last error code")
eq(rec(0), 1, "a corrupt CRC: REC")
eq(ev(0, "rxok"), 0, "a corrupt CRC: rxok")
eq(ev(0, "err"), 1, "a corrupt CRC: err")

# CRC 定界符是显性：格式错误，标志从下一位起
dl = len(ext["body"])
scripted("a dominant CRC delimiter", ext["body"] + tail(1, crcdel=0))
seen(dl + 1, dl + 6, 0, "a dominant CRC delimiter: no error flag from the next bit")
eq(lec(0), 2, "a dominant CRC delimiter: last error code")
eq(rec(0), 2, "a dominant CRC delimiter: REC")

# 去掉数据场里五个隐性之后的填充位：第 6 个隐性就是填充错误
ones = frame(0, 0, 0x055 << 18, 2, [0xFF, 0xFF])
miss = first(ones, lambda b, k, h: k < 0 and b == 0 and -k - 1 >= h)
# 真的发送方见到错误标志会停下来发自己的标志；脚本节点在出错那一位之后就放开总线，免得它接着发的显性位
# 在被测件的定界符里再引出格式错误和规则 2
scripted("six recessive bits in the data field", (ones["body"][:miss] + ones["body"][miss + 1:])[:miss + 1] + [1] * 20)
seen(miss + 1, miss + 6, 0, "six recessive bits: no error flag from the next bit")
eq(lec(0), 1, "six recessive bits: last error code")
eq(rec(0), 3, "six recessive bits: REC")

# 验收滤波：应答不看滤波，只管进不进缓冲；两次成功接收各减一
other = frame(0, 0, 0x124 << 18, 1, [0x55])
if filt:
    emit(f"wr(0, 8'h{FID:02X}, 32'h{0x123 << 18:08X});")
    emit(f"wr(0, 8'h{FMASK:02X}, 32'h{0x7FF << 18:08X});")
name = "a frame the filter rejects" if filt else "a frame with the filter off"
ack = len(other["body"]) + 1
scripted(name, other["body"] + tail(1))
seen(ack, ack, 0, f"{name}: node 0 did not acknowledge it")
eq(ev(0, "rxok"), 0 if filt else 1, f"{name}: rxok")
if not filt:
    rx_is(0, other, name)
eq(rec(0), 2, f"{name}: REC")
ack = len(std["body"]) + 1
scripted("a frame the filter accepts", std["body"] + tail(1))
seen(ack, ack, 0, "a frame the filter accepts: node 0 did not acknowledge it")
eq(ev(0, "rxok"), 1, "a frame the filter accepts: rxok")
rx_is(0, std, "a frame the filter accepts")
eq(rec(0), 1, "a frame the filter accepts: REC")
if filt:
    emit(f"wr(0, 8'h{FMASK:02X}, 0);")

# 脚本节点每 5 位里有一位短一拍：不再同步就会错位
ack = len(ext["body"]) + 1
scripted("an extended frame from a node 0.5% fast", ext["body"] + tail(1), jitter=True)
seen(ack, ack, 0, "a fast node: node 0 did not acknowledge it")
rx_is(0, ext, "a fast node")
eq(rec(0), 0, "a fast node: REC")

# ================= 0 号发，测试台监视并应答 =================
for name, f in (("sending a standard data frame", frame(0, 0, 0x2A5 << 18, 4, [0xDE, 0xAD, 0xBE, 0xEF])),
                ("sending an extended remote frame", frame(1, 1, 0x0F0F0F0, 5))):
    ack = len(f["body"]) + 1
    expect = f["body"] + [1, 0, 1] + [1] * 7
    val = sum(b << i for i, b in enumerate(expect))
    mask = (1 << len(expect)) - 1
    emit(f"// ---- {name} ----")
    emit(f"wr(0, 8'h{EVENTS:02X}, 32'h7F);")
    emit(f"action ackMode <= 1; ackIdx <= {ack}; endaction")
    load(0, f)
    emit(f"wr(0, 8'h{CMD:02X}, 1);")
    emit("delay(3);")
    emit(f'waitIdle(0, "{name}");')
    emit("action ackMode <= 0; endaction")
    emit("snap(0);")
    check(f"(mon & 256'h{mask:064X}) == 256'h{val:064X}", f"{name}: the bus carried %h (bit 0 first)", "mon")
    eq(tec(0), 0, f"{name}: TEC")
    eq(ev(0, "txok"), 1, f"{name}: txok")
    eq(ev(0, "err"), 0, f"{name}: err")

# ================= 两个节点：仲裁与注入 =================
emit(f"wr(1, 8'h{CTRL:02X}, 1);")
emit("delay(600);")
emit('waitIdle(1, "after enabling node 1");')

for name, f0, f1 in (("standard 0x123 against standard 0x456",
                      frame(0, 0, 0x456 << 18, 1, [0x11]), frame(0, 0, 0x123 << 18, 1, [0x22])),
                     ("a standard frame against an extended frame with the same base identifier",
                      frame(1, 0, (0x123 << 18) | 0x2AB, 1, [0x33]), frame(0, 0, 0x123 << 18, 1, [0x44]))):
    emit(f"// ---- {name} ----")
    emit(f"wrBoth(8'h{EVENTS:02X}, 32'h7F);")
    load(0, f0)
    load(1, f1)
    emit(f"wrBoth(8'h{CMD:02X}, 1);")
    emit("delay(3);")
    emit(f'waitBoth("{name}");')
    emit("snap(0);")
    emit("snap(1);")
    eq(ev(0, "arblost"), 1, f"{name}: node 0 did not lose arbitration")
    eq(ev(1, "arblost"), 0, f"{name}: node 1 lost arbitration")
    rx_is(0, f1, f"{name}: node 0 receive buffer")
    rx_is(1, f0, f"{name}: node 1 receive buffer")
    for n in (0, 1):
        eq(ev(n, "txok"), 1, f"{name}: node {n} txok")
        eq(tec(n), 0, f"{name}: node {n} TEC")

# 注入：节点 1 发，数据场里第一个填充位（隐性，前面 5 个显性）被拉成显性
inj = frame(0, 0, 0x100 << 18, 1, [0x00])
k = first(inj, lambda b, kk, h: kk < 0 and b == 1 and -kk - 1 >= h)
name = "a stuff bit forced dominant"
emit(f"// ---- {name} ----")
emit(f"wrBoth(8'h{EVENTS:02X}, 32'h7F);")
load(1, inj)
emit(f"action injIdx <= {k}; injOn <= True; runBase <= sofN; endaction")
emit(f"wr(1, 8'h{CMD:02X}, 1);")
emit("await(sofN != runBase);")
emit(f"delay({(k + 2) * 40});")
emit("action injOn <= False; endaction")
emit(f'waitBoth("{name}");')
emit("snap(0);")
emit("snap(1);")
eq(lec(0), 1, f"{name}: node 0 last error code")
eq(lec(1), 4, f"{name}: node 1 last error code")
eq(tec(1), 7, f"{name}: node 1 TEC after the error and the retransmission")
eq(rec(0), 0, f"{name}: node 0 REC after the error and the retransmission")
rx_is(0, inj, name)
eq(ev(0, "rxok"), 1, f"{name}: node 0 rxok")
eq(ev(0, "rxover"), 0, f"{name}: node 0 received the frame twice")
eq(ev(0, "err"), 1, f"{name}: node 0 err")
eq(ev(1, "err"), 1, f"{name}: node 1 err")
eq(ev(1, "txok"), 1, f"{name}: node 1 txok")

# ================= 只剩 0 号：应答错误走到消极 =================
emit(f"wr(1, 8'h{CTRL:02X}, 0);")
lone = next(fr for v in range(256) for fr in [frame(0, 0, 0x333 << 18, 1, [v])] if fr["body"][-1] == 0)
ack = len(lone["body"]) + 1
name = "twenty unacknowledged attempts"
emit(f"// ---- {name} ----")
emit(f"wr(0, 8'h{EVENTS:02X}, 32'h7F);")
emit(f"action runBase <= sofN; ackMode <= 2; ackN <= 21; ackIdx <= {ack}; endaction")
load(0, lone)
emit(f"wr(0, 8'h{CMD:02X}, 1);")
emit("delay(3);")
emit(f'waitIdle(0, "{name}");')
emit("action ackMode <= 0; endaction")
emit("snap(0);")
# 主动：6 位标志之后定界符 8 加间歇 3；刚变消极多 8 位暂停发送；
# 消极：CRC 定界符、应答位、6 位消极标志、定界符 8、间歇 3、暂停发送 8
for j in range(2, 17):
    rng(f"runs[{j}]", 11 * 40 - 2, 11 * 40 + 2, f"recessive bits before attempt {j}, want 11")
rng("runs[17]", 19 * 40 - 2, 19 * 40 + 2, "recessive bits before attempt 17, want 19")
for j in range(18, 22):
    rng(f"runs[{j}]", 27 * 40 - 2, 27 * 40 + 2, f"recessive bits before attempt {j}, want 27")
eq(tec(0), 127, f"{name}: TEC after twenty failures and one success")
eq(state(0), 0, f"{name}: state after the success")
eq(ev(0, "passive"), 1, f"{name}: passive")
eq(ev(0, "txok"), 1, f"{name}: txok")

# ================= 总线关闭与恢复 =================
off = frame(0, 0, 0x3A0 << 18, 2, [0xF0, 0x0F])
k = first(off, lambda b, kk, h: kk >= h and b == 1)
name = "bus off"
emit(f"// ---- {name} ----")
emit(f"wr(0, 8'h{EVENTS:02X}, 32'h7F);")
emit(f"action runBase <= sofN; injIdx <= {k}; injOn <= True; endaction")
load(0, off)
emit(f"wr(0, 8'h{CMD:02X}, 1);")
emit("action t0 <= cyc; rdv <= 0; endaction")
emit(f"while (rdv[1:0] != 2 && cyc - t0 < 400000) rd(0, 8'h{STATUS:02X});")
emit("action boAt <= cyc; injOn <= False; boWatch <= True; endaction")
emit("snap(0);")
eq(state(0), 2, "bus off: state")
eq(tec(0), 256, "bus off: TEC")
eq(txpend(0), 0, "bus off: the pending frame was not withdrawn")
eq(ev(0, "busoff"), 1, "bus off: busoff")
emit("action rdv <= 2; endaction")
emit(f"while (rdv[1:0] != 0 && cyc - boAt < 80000) rd(0, 8'h{STATUS:02X});")
emit("action boWatch <= False; boGot <= cyc - boAt; endaction")
rng("boGot", 1407 * 40, 1409 * 40, "cycles from bus off to error active, want 128 x 11 bits")
emit("snap(0);")
eq(tec(0), 0, "after recovery: TEC")
eq(rec(0), 0, "after recovery: REC")
eq("pack(boTx)", 0, "bus off: can_tx went dominant while bus off")
flush()

script_cases = "\n".join(
    f"      {i}: return 256'h{(sum(b << j for j, b in enumerate(s)) | (((1 << 256) - 1) ^ ((1 << len(s)) - 1))):064X};"
    for i, s in enumerate(scripts))
script_lens = "\n".join(f"      {i}: return {len(s)};" for i, s in enumerate(scripts))
if max(len(s) for s in scripts) > 255:
    raise SystemExit("脚本位流超过 255 位")

verdict = ("standard, extended and remote frames are received and acknowledged, a corrupt CRC, a dominant CRC delimiter "
           "and six equal bits each raise their flag on the right bit with the right error code and REC, "
           + ("the acceptance filter keeps a rejected frame out of the buffer but still acknowledges it, "
              if filt else "every frame is received with the filter off, ")
           + "a node 0.5% fast is followed by resynchronization, sent frames match the reference bit stream bit for bit, "
           "the lower identifier and the standard frame win arbitration and the loser retransmits, a forced stuff bit "
           "gives a stuff error at the receiver and a bit error at the transmitter, unacknowledged retries pass "
           "11, 19 and 27 recessive bits and stop at TEC 128, and bus off recovers after 128 x 11 recessive bits")

TEMPLATE = r'''package Can@L@Tb;

// 由 htest/mkcantb.py 生成，勿手改。这一点：filter=@FILT@

import StmtFSM::*;
import Vector::*;
import ConfigReg::*;
import RegIf::*;
import Can::*;

(* synthesize *)
module mkCan@L@Tb(Empty);
  CanIfc#(8, 32) n0 <- mkCan(CanCfg { filter: @FILTB@ });
  CanIfc#(8, 32) n1 <- mkCan(CanCfg { filter: @FILTB@ });
  Vector#(2, CanIfc#(8, 32)) nd = cons(n0, cons(n1, nil));

  Reg#(UInt#(32)) cyc <- mkConfigReg(0);

  // ---- 测试序列写、总线读的控制量 ----
  Reg#(UInt#(4)) scrCase <- mkConfigReg(0);
  Reg#(UInt#(8)) scrGo   <- mkConfigReg(0);
  Reg#(Bool)     jitter  <- mkConfigReg(False);
  Reg#(UInt#(2)) ackMode <- mkConfigReg(0);   // 0 不应答 · 1 每帧应答 · 2 只应答第 ackN 次
  Reg#(UInt#(8)) ackN    <- mkConfigReg(0);
  Reg#(UInt#(8)) ackIdx  <- mkConfigReg(0);
  Reg#(Bool)     injOn   <- mkConfigReg(False);
  Reg#(UInt#(8)) injIdx  <- mkConfigReg(0);
  Reg#(UInt#(8)) runBase <- mkConfigReg(0);
  Reg#(Bool)     boWatch <- mkConfigReg(False);

  // ---- 总线写、测试序列读的状态 ----
  Reg#(Bit#(1))   drvR    <- mkConfigReg(1);
  Reg#(Bit#(1))   injR    <- mkConfigReg(0);
  Reg#(Bit#(1))   prevBus <- mkReg(1);
  Reg#(UInt#(32)) recRun  <- mkConfigReg(0);
  Reg#(UInt#(8))  sofN    <- mkConfigReg(0);
  Reg#(UInt#(32)) sofAt   <- mkConfigReg(0);
  Vector#(24, Reg#(UInt#(32))) runs <- replicateM(mkConfigReg(0));
  Reg#(Bit#(256)) mon     <- mkConfigReg('1);
  Reg#(UInt#(8))  scrTok  <- mkReg(0);
  Reg#(Bool)      scrOn   <- mkConfigReg(False);
  Reg#(UInt#(8))  scrI    <- mkReg(0);
  Reg#(UInt#(8))  scrPh   <- mkReg(0);
  Reg#(Bit#(256)) seen    <- mkConfigReg('1);
  Reg#(UInt#(32)) boTx    <- mkConfigReg(0);

  Bit#(1) bus = n0.pins.can_tx & n1.pins.can_tx & drvR & ~injR;

  function Bit#(256) script(UInt#(4) c);
    case (c)
@SCRIPTS@
      default: return '1;
    endcase
  endfunction

  function UInt#(8) scriptLen(UInt#(4) c);
    case (c)
@SCRLENS@
      default: return 0;
    endcase
  endfunction

  rule wire_;
    n0.pins.can_rx(bus);
    n1.pins.can_rx(bus);
  endrule

  rule bench;
    cyc <= cyc + 1;
    if (cyc > @LIMIT@) begin
      $display("TIMEOUT");
      $finish(1);
    end

    // 帧起始：至少 10 位隐性之后的下降沿（帧里填充保证不会连着 6 位以上）
    Bool      fall = bus == 0 && prevBus == 1;
    Bool      sof  = fall && recRun >= 400;
    UInt#(32) rel  = cyc - sofAt;
    UInt#(32) bitI = rel / 40;
    UInt#(32) ph   = rel % 40;
    prevBus <= bus;
    recRun  <= bus == 1 ? recRun + 1 : 0;
    if (sof) begin
      sofN  <= sofN + 1;
      sofAt <= cyc;
      UInt#(8) k = sofN + 1 - runBase;
      if (k < 24) runs[k] <= recRun;
      mon <= '1;
    end else if (ph == 20 && bitI < 256 && bus == 0) begin
      mon <= mon & ~(1 << bitI);
    end

    Bool ackNow = !sof && bitI == zeroExtend(ackIdx) &&
                  (ackMode == 1 || (ackMode == 2 && sofN - runBase == ackN));
    Bool injNow = !sof && injOn && bitI == zeroExtend(injIdx) && ph >= 4 && ph < 38;

    Bit#(1) sb = 1;
    if (scrTok != scrGo) begin
      scrTok <= scrGo; scrOn <= True; scrI <= 0; scrPh <= 0; seen <= '1;
    end else if (scrOn) begin
      sb = script(scrCase)[scrI];
      UInt#(8) btNow = (jitter && scrI % 5 == 4) ? 39 : 40;
      if (scrPh == 30 && bus == 0) seen <= seen & ~(1 << scrI);
      if (scrPh + 1 >= btNow) begin
        scrPh <= 0;
        scrI  <= scrI + 1;
        if (scrI + 1 >= scriptLen(scrCase)) scrOn <= False;
      end else scrPh <= scrPh + 1;
    end
    drvR <= ackNow ? 0 : sb;
    injR <= injNow ? 1 : 0;
    if (boWatch && n0.pins.can_tx == 0) boTx <= boTx + 1;
  endrule

  // ---- 测试序列 ----
  Reg#(Bool)      bad   <- mkReg(False);
  Reg#(Bit#(32))  rdv   <- mkReg(0);
  Reg#(Bit#(32))  s0    <- mkReg(0);
  Reg#(Bit#(32))  s1    <- mkReg(0);
  Reg#(UInt#(32)) t0    <- mkReg(0);
  Reg#(UInt#(32)) boAt  <- mkReg(0);
  Reg#(UInt#(32)) boGot <- mkReg(0);
  // 每个节点读回来的 0 status · 1 errcnt · 2 rxid · 3 rxdlc · 4 rxd0 · 5 rxd1 · 6 events
  Vector#(2, Vector#(7, Reg#(Bit#(32)))) sn <- replicateM(replicateM(mkReg(0)));

  function Action wr(Integer n, Bit#(8) a, Bit#(32) v) = action
    let x <- nd[n].regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;

  function Action wrBoth(Bit#(8) a, Bit#(32) v) = action
    let x <- n0.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
    let y <- n1.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;

  function Action rd(Integer n, Bit#(8) a) = action
    let x <- nd[n].regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    rdv <= x.rdata;
  endaction;

  function Action peek(Integer n, Integer i, Bit#(8) a) = action
    let x <- nd[n].regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    sn[n][i] <= x.rdata;
  endaction;

  function Stmt snap(Integer n) = seq
    peek(n, 0, 8'h08); peek(n, 1, 8'h0C); peek(n, 2, 8'h24); peek(n, 3, 8'h28);
    peek(n, 4, 8'h2C); peek(n, 5, 8'h30); peek(n, 6, 8'h3C);
  endseq;

  // 空闲且没有待发帧才算这一步走完
  function Stmt waitIdle(Integer n, String what) = seq
    action t0 <= cyc; rdv <= 0; endaction
    while ((rdv[3] == 0 || rdv[2] == 1) && cyc - t0 < 300000) rd(n, 8'h08);
    action if (cyc - t0 >= 300000) begin $display("FAIL node %0d never went idle: %s", n, what); bad <= True; end endaction
  endseq;

  function Stmt waitBoth(String what) = seq
    action t0 <= cyc; s0 <= 0; s1 <= 0; endaction
    while ((s0[3] == 0 || s0[2] == 1 || s1[3] == 0 || s1[2] == 1) && cyc - t0 < 300000) seq
      action let x <- n0.regs.access(RegReq { addr: 8'h08, write: False, wdata: 0, wstrb: 4'hF }); s0 <= x.rdata; endaction
      action let x <- n1.regs.access(RegReq { addr: 8'h08, write: False, wdata: 0, wstrb: 4'hF }); s1 <= x.rdata; endaction
    endseq
    action if (cyc - t0 >= 300000) begin $display("FAIL the two nodes never both went idle: %s", what); bad <= True; end endaction
  endseq;

  Stmt test = seq
@STEPS@
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule fin (started && fsm.done);
    if (bad) $display("FAILED");
    else $display("PASS can: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label)
       .replace("@FILT@", str(filt))
       .replace("@FILTB@", "True" if filt else "False")
       .replace("@SCRIPTS@", script_cases)
       .replace("@SCRLENS@", script_lens)
       .replace("@LIMIT@", "900000")
       .replace("@STEPS@", "\n".join(S))
       .replace("@VERDICT@", verdict))

(out / f"Can{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  can 行为测试台就位：filter={filt}，{len(scripts)} 段脚本位流，最长 {max(len(s) for s in scripts)} 位")
