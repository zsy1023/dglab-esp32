#!/usr/bin/env python3
"""dglab-esp32 上位机示例：**实时读取**郊狼强度 / 波形 / 回路，并能反向控制。

只依赖 pyserial（``pip install pyserial``）。适用于任何接了本固件的 ESP32 板子。

快速上手
--------

    # 1) 看本机有哪些串口（会自动标出最像 ESP32 的那个）
    python host_example.py --list

    # 2) 实时打印（Ctrl+C 退出）——这就是"实时获取强度"
    python host_example.py --port COM19

    # 3) 一次性控制：把 A 强度设成 20、把 B 的回路断开，看 5 秒就退出
    python host_example.py --port COM19 --set A=20 --loop B=off --for 5

    # 4) 没有板子也能验证解析逻辑
    python host_example.py --selftest

它在读什么
----------

固件把状态打成 **纯 ASCII 的 ``#`` 文本行**（100ms 一条 ``#B0``，1 秒一条 ``#STAT``），
这就是"实时"的来源 —— 每帧 4 组 × 25ms 的波形都在里面::

    #B0 t=1520 seq=0 mode=A./B. set=0/0 A=5/150 B=0/30 link=A1/B1 \
        wA=10:0,10:10,20:20,30:30 wB=!0:0,0:0,0:0,0:101 \
        raw=B00000000A0A0A0A000A141E0000000000000065
    #STAT t=2620 A=5/150 B=0/30 link=A1/B1 ... | b0=12 (10.0/s) ... conn=1 sub=1 raw=0 verbose=1
    #B1 -> App seq=0 A=5 B=0
    #STRENGTH A=5/150 B=0/30 (seq=1)
    #link: loop A=closed B=open | strength A=5/150 B=0/30

字段含义：

* ``A=5/150`` —— A 通道**当前强度 / 软上限**（0~200）；B 同理
* ``link=A1/B1`` —— 回路是否闭合（1 闭合 / 0 断开）
* ``wA=`` / ``wB=`` —— 该通道 4 组 ``频率:强度``（频率 10~240Hz，强度 0~100，每组管 25ms）
* ``wA=`` 前面带 ``!`` —— **整组作废**：真机不输出该通道（频率越界或强度 >100，App 常用 101 静音）
* ``conn`` / ``sub`` —— 手机 App 有没有连上 / 有没有订阅 0x150B
* ``b0=12 (10.0/s)`` —— 最近 1 秒收到多少条 B0（正常 10/s，掉帧一眼看得出）

能发什么（反向控制）
--------------------

全是 ``HEAD + 通道 + 参数`` 的 3 字节帧，通道 ``0``=A、``1``=B、``0xFF``=两个都改::

    0xD0 ch value   强度绝对值（>200 按 200，仍受软上限夹值）
    0xD1 ch delta   强度相对增减（int8：0x01=+1，0xFF=-1）
    0xD2 ch on      回路开关（0=断开 / 非 0=闭合）

固件收到强度变化会回一条 ``B1(seq=0)``，所以**手机上 App 的显示会同步**，不会跟网页打架。

两个坑（都踩过）
----------------

1. **串口是独占的**：串口监视器 / webDG / 本示例只能有一个在跑。
2. 打开串口时把 **DTR/RTS 压成 False**，否则 CH340/CP2102 会把板子复位（本示例已处理）。
   另外：本示例**不发 ``0xCE``** —— 那个字节会让固件把原始二进制帧也上行出来，污染文本解析。
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - 只在没装依赖时走到
    sys.exit("需要 pyserial：pip install pyserial")

# ══════════════════════════════════════════════════════════════
# 串口协议常量（与 firmware/src/main.cpp 一致）
# ══════════════════════════════════════════════════════════════
BAUDRATE = 115200

CMD_RAW_OFF = 0xC0        # 关掉原始 B0/BF 二进制上行（只要文本）
CMD_RAW_ON = 0xC1
CMD_VERBOSE_OFF = 0xC2    # 关掉逐帧 #B0 日志
CMD_VERBOSE_ON = 0xC3     # 打开逐帧 #B0 日志（实时波形就靠它）
CMD_STRENGTH_SET = 0xD0   # D0 ch value
CMD_STRENGTH_STEP = 0xD1  # D1 ch delta(int8)
CMD_LOOP = 0xD2           # D2 ch on
CH_A = 0
CH_B = 1
CH_BOTH = 0xFF

#: 固件的帧率（B0 每 100ms 一条）
FRAME_HZ = 10.0


# ══════════════════════════════════════════════════════════════
# 状态
# ══════════════════════════════════════════════════════════════
@dataclass
class ChannelState:
    """一个通道的实时状态。"""

    strength: int = 0            # 当前强度 0~200
    cap: int = 200               # 软上限（BF 设定，断电保存）
    loop: bool = True            # 回路闭合？（断开 = 真机没电流）
    wave: list[tuple[int, int]] = field(default_factory=list)  # 4 组 (频率, 强度)
    wave_valid: bool = False     # False = 整组作废（真机不输出该通道）
    wave_seen: bool = False      # 有没有收到过波形数据

    @property
    def peak(self) -> int:
        """4 组里最大的波形强度。"""
        return max((a for _, a in self.wave), default=0)

    @property
    def outputting(self) -> bool:
        """现在到底有没有在输出：回路闭合 + 波形合法 + 至少一组强度 >0。"""
        return self.loop and self.wave_valid and self.peak > 0

    def wave_text(self) -> str:
        if not self.wave_seen:
            return "-"
        if not self.wave_valid:
            return "!" + ",".join(f"{f}:{a}" for f, a in self.wave)
        return ",".join(f"{f}:{a}" for f, a in self.wave)


@dataclass
class HostState:
    """一台假主机（dglab-esp32）的实时状态快照。"""

    a: ChannelState = field(default_factory=ChannelState)
    b: ChannelState = field(default_factory=ChannelState)
    connected: bool = False       # 串口链路有没有打开
    app_connected: bool = False   # 手机 App 有没有连上（固件报的 conn）
    app_subscribed: bool = False  # 手机 App 有没有订阅 0x150B（订阅了才会回 B1）
    b0_rate: float = 0.0          # 最近 1 秒的 B0 帧率（正常 10/s）
    b1_count: int = 0             # 收到过多少条 B1 回包
    last_b1: tuple[int, int, int] | None = None   # (seq, A, B)
    last_frame_ts: float = 0.0    # 最后一条 B0 的本地时间（用来判断是不是掉线）

    def channel(self, ch: int) -> ChannelState:
        return self.a if ch == CH_A else self.b

    def summary(self) -> str:
        """一行看清楚：强度 / 软上限 / 回路 / 波形 / App 状态。"""
        def one(name: str, st: ChannelState) -> str:
            link = "闭环" if st.loop else "断开"
            wave = st.wave_text()
            shot = ""
            if st.wave_seen and st.wave_valid:
                shot = f" 峰{st.peak:>3}"
            return f"{name} {st.strength:>3}/{st.cap:<3} [{link}] w{name}={wave}{shot}"

        app = "连上" if self.app_connected else "没连"
        sub = "已订阅" if self.app_subscribed else "未订阅"
        return (
            f"{one('A', self.a)} | {one('B', self.b)} | "
            f"app={app}/{sub} b0={self.b0_rate:.1f}/s"
        )


# ══════════════════════════════════════════════════════════════
# 解析固件的 '#' 日志行
# ══════════════════════════════════════════════════════════════
_RE_B0 = re.compile(
    r"^#B0\s+t=(?P<t>\d+)\s+seq=(?P<seq>\d+)\s+mode=A(?P<ma>\S)/B(?P<mb>\S)"
    r"\s+set=(?P<sa>\d+)/(?P<sb>\d+)"
    r"\s+A=(?P<a>\d+)/(?P<acap>\d+)\s+B=(?P<b>\d+)/(?P<bcap>\d+)"
    r"\s+link=A(?P<la>\d)/B(?P<lb>\d)"
    r"\s+wA=(?P<wa>\S+)\s+wB=(?P<wb>\S+)"
    r"(?:\s+raw=(?P<raw>\S+))?"
)
_RE_STAT = re.compile(
    r"^#STAT\s+t=(?P<t>\d+)\s+A=(?P<a>\d+)/(?P<acap>\d+)\s+B=(?P<b>\d+)/(?P<bcap>\d+)"
    r"\s+link=A(?P<la>\d)/B(?P<lb>\d)"
)
_RE_STAT_EXTRA = re.compile(r"b0=(?P<b0>\d+)\s+\((?P<rate>[\d.]+)/s\).*?"
                            r"conn=(?P<conn>\d)\s+sub=(?P<sub>\d)")
_RE_STRENGTH = re.compile(r"^(?:#)?STRENGTH\s+A=(\d+)/(\d+)\s+B=(\d+)/(\d+)")
_RE_LOOP = re.compile(r"^#?link: loop A=(?P<a>closed|open) B=(?P<b>closed|open)")
_RE_B1 = re.compile(r"^#B1 -> App seq=(?P<seq>\d+) A=(?P<a>\d+) B=(?P<b>\d+)")


def parse_wave_cell(text: str) -> tuple[bool, list[tuple[int, int]]]:
    """``wA=`` 的值 → ``(是否合法, [(频率, 强度), ...])``。

    固件在整组作废时会在前面加 ``!``（数字照样给），例如 ``!0:0,0:0,0:0,0:101``。
    组数不是 4 视为这条日志残缺 → ``(False, [])``。
    """
    if not text:
        return False, []
    valid = not text.startswith("!")
    body = text[1:] if not valid else text
    groups: list[tuple[int, int]] = []
    for part in body.split(","):
        if ":" not in part:
            return False, []
        f, a = part.split(":", 1)
        try:
            groups.append((int(f), int(a)))
        except ValueError:
            return False, []
    if len(groups) != 4:
        return False, []
    return valid, groups


def apply_line(state: HostState, line: str) -> str | None:
    """把一条固件日志应用进 ``state``；返回"值得打印的事件"（没有就 None）。"""
    line = line.rstrip("\r\n")
    if not line:
        return None

    if line.startswith("#B0 "):
        m = _RE_B0.match(line)
        if not m:
            return None
        state.a.strength = int(m["a"])
        state.a.cap = int(m["acap"])
        state.b.strength = int(m["b"])
        state.b.cap = int(m["bcap"])
        state.a.loop = m["la"] == "1"
        state.b.loop = m["lb"] == "1"
        state.a.wave_valid, state.a.wave = parse_wave_cell(m["wa"])
        state.b.wave_valid, state.b.wave = parse_wave_cell(m["wb"])
        state.a.wave_seen = state.b.wave_seen = True
        state.last_frame_ts = time.monotonic()
        return None

    if line.startswith("#STAT "):
        m = _RE_STAT.match(line)
        if m:
            state.a.strength, state.a.cap = int(m["a"]), int(m["acap"])
            state.b.strength, state.b.cap = int(m["b"]), int(m["bcap"])
            state.a.loop = m["la"] == "1"
            state.b.loop = m["lb"] == "1"
        extra = _RE_STAT_EXTRA.search(line)
        if extra:
            state.b0_rate = float(extra["rate"])
            state.app_connected = extra["conn"] == "1"
            state.app_subscribed = extra["sub"] == "1"
        return None

    m = _RE_STRENGTH.match(line)
    if m:
        before = (state.a.strength, state.b.strength)
        state.a.strength, state.a.cap = int(m.group(1)), int(m.group(2))
        state.b.strength, state.b.cap = int(m.group(3)), int(m.group(4))
        if before != (state.a.strength, state.b.strength):
            return f"强度变化 → A={state.a.strength}/{state.a.cap} B={state.b.strength}/{state.b.cap}"
        return None

    m = _RE_LOOP.match(line)
    if m:
        state.a.loop = m["a"] == "closed"
        state.b.loop = m["b"] == "closed"
        return f"回路 → A={'闭合' if state.a.loop else '断开'} B={'闭合' if state.b.loop else '断开'}"

    m = _RE_B1.match(line)
    if m:
        state.b1_count += 1
        state.last_b1 = (int(m["seq"]), int(m["a"]), int(m["b"]))
        return f"收到 B1：seq={m['seq']} A={m['a']} B={m['b']}"

    return None


# ══════════════════════════════════════════════════════════════
# 串口链路
# ══════════════════════════════════════════════════════════════
class DglabLink:
    """到 dglab-esp32 的一条串口链路：后台线程读，``.state`` 随时读。

    :param port: 串口名（``COM19`` / ``/dev/ttyUSB0``）
    :param on_event: 状态变化时的回调（强度变化 / 回路 / B1），在**读线程**里被调用
    """

    def __init__(self, port: str, baudrate: int = BAUDRATE,
                 *, on_event: Callable[[str], None] | None = None) -> None:
        self.port = port
        self.baudrate = baudrate
        self.state = HostState()
        self.on_event = on_event
        self._ser: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    # ── 生命周期 ──────────────────────────────────────────
    def open(self) -> "DglabLink":
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baudrate
        ser.timeout = 0.2          # 读超时：让读线程能定期看停止标志
        ser.write_timeout = 1.0
        ser.dtr = False            # ★ 打开前就压住 DTR/RTS，否则板子会被复位
        ser.rts = False
        ser.open()
        try:
            ser.dtr = False
            ser.rts = False
            ser.reset_input_buffer()   # 丢掉开机残渣
        except Exception:  # noqa: BLE001 - 某些驱动不支持，忽略
            pass
        self._ser = ser
        self.state.connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="dglab-reader", daemon=True)
        self._thread.start()
        # 要"实时"就要逐帧日志；同时关掉原始二进制帧（会污染文本）
        self.send_verbose(True)
        self.send_raw(False)
        return self

    def close(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        ser, self._ser = self._ser, None
        self.state.connected = False
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def __enter__(self) -> "DglabLink":
        return self.open()

    def __exit__(self, *_exc: object) -> None:
        self.close()

    # ── 控制（发字节）────────────────────────────────────
    def write(self, data: Iterable[int]) -> None:
        ser = self._ser
        if ser is None:
            raise RuntimeError("链路没打开")
        with self._write_lock:
            ser.write(bytes(data))

    def send_verbose(self, on: bool) -> None:
        """是否逐帧打 ``#B0``（实时波形靠它，默认开）。"""
        self.write([CMD_VERBOSE_ON if on else CMD_VERBOSE_OFF])

    def send_raw(self, on: bool) -> None:
        """是否把原始 ``B0``/``BF`` 二进制帧也上行（默认关）。"""
        self.write([CMD_RAW_ON if on else CMD_RAW_OFF])

    def set_strength(self, ch: int, value: int) -> None:
        """强度绝对值（0~200，仍受软上限夹值）。"""
        self.write([CMD_STRENGTH_SET, ch & 0xFF, max(0, min(200, int(value)))])

    def step_strength(self, ch: int, delta: int) -> None:
        """强度相对增减（点动用 ±1）。"""
        self.write([CMD_STRENGTH_STEP, ch & 0xFF, int(delta) & 0xFF])

    def set_loop(self, ch: int, on: bool) -> None:
        """回路开关（0 = 断开 / 1 = 闭合）。"""
        self.write([CMD_LOOP, ch & 0xFF, 1 if on else 0])

    # ── 读线程 ───────────────────────────────────────────
    def _reader(self) -> None:
        ser = self._ser
        while not self._stop.is_set() and ser is not None:
            try:
                raw = ser.readline()
            except Exception as exc:  # noqa: BLE001 - 拔线 / 端口被抢占
                if not self._stop.is_set():
                    self._emit(f"串口读失败（链路断开）：{exc}")
                self.state.connected = False
                return
            if not raw:
                continue
            event = apply_line(self.state, raw.decode("utf-8", "replace"))
            if event:
                self._emit(event)

    def _emit(self, text: str) -> None:
        if self.on_event is None:
            return
        try:
            self.on_event(text)
        except Exception:  # noqa: BLE001 - 回调出错不能把读线程打死
            pass


# ══════════════════════════════════════════════════════════════
# 命令行
# ══════════════════════════════════════════════════════════════
def cmd_list() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到任何串口（驱动装了吗？板子插了吗？）")
        return 1
    for p in ports:
        hint = ""
        if p.vid is not None:
            name = {0x1A86: "WCH CH34x", 0x10C4: "CP210x", 0x0403: "FTDI"}.get(p.vid, f"vid={p.vid:04X}")
            hint = f"  [{name}]"
        if "BTHENUM" in (p.hwid or ""):
            hint += "  ← 蓝牙虚拟口，能打开但收不到字节，别选"
        print(f"{p.device:<10} {p.description}{hint}")
    return 0


def cmd_selftest() -> int:
    """不接板子也能验证解析：喂固件真实格式的日志行，检查状态。"""
    state = HostState()
    lines = [
        "#dglab-esp32 boot: name=47L121000 bt_mac=EC:E3:34:A9:13:0E",
        "#App connected (conn_id=0) | strength A=0/150 B=0/30",
        "#B0 t=1520 seq=0 mode=A./B. set=0/0 A=5/150 B=0/30 link=A1/B1 "
        "wA=10:0,10:10,20:20,30:30 wB=!0:0,0:0,0:0,0:101 "
        "raw=B00000000A0A0A0A000A141E0000000000000065",
        "#STAT t=2620 A=5/150 B=0/30 link=A1/B1 wA=10:0,10:10,20:20,30:30 "
        "wB=!0:0,0:0,0:0,0:101 | b0=12 (10.0/s) bf=1 b1=1 drop=0 down=0 logdrop=0 "
        "conn=1 sub=1 raw=0 verbose=1",
        "#B1 -> App seq=3 A=7 B=0",
        "#STRENGTH A=7/150 B=0/30 (seq=3)",
        "#link: loop A=closed B=open | strength A=7/150 B=0/30",
        "garbage line without hash",
        "",
    ]
    events = [ev for ev in (apply_line(state, ln) for ln in lines) if ev]

    problems: list[str] = []
    if (state.a.strength, state.a.cap) != (7, 150):
        problems.append(f"A 强度应为 7/150，实际 {state.a.strength}/{state.a.cap}")
    if (state.b.strength, state.b.cap) != (0, 30):
        problems.append(f"B 强度应为 0/30，实际 {state.b.strength}/{state.b.cap}")
    if state.a.loop is not True or state.b.loop is not False:
        problems.append(f"回路应为 A 闭合 / B 断开，实际 {state.a.loop}/{state.b.loop}")
    if state.a.wave != [(10, 0), (10, 10), (20, 20), (30, 30)] or not state.a.wave_valid:
        problems.append(f"A 波形解析不对：{state.a.wave} valid={state.a.wave_valid}")
    if state.b.wave_valid is not False:
        problems.append("B 的 '!' 前缀应判为整组作废")
    if state.b.wave != [(0, 0), (0, 0), (0, 0), (0, 101)]:
        problems.append(f"B 波形数字应保留：{state.b.wave}")
    if not state.app_connected or not state.app_subscribed:
        problems.append("conn/sub 应为 1/1")
    if abs(state.b0_rate - 10.0) > 1e-6:
        problems.append(f"帧率应为 10.0，实际 {state.b0_rate}")
    if state.b1_count != 1 or state.last_b1 != (3, 7, 0):
        problems.append(f"B1 应记 1 条 (3,7,0)，实际 {state.b1_count} {state.last_b1}")
    if not state.a.outputting:
        problems.append("A 应该在输出（回路闭合 + 波形合法 + 峰值 >0）")
    if state.b.outputting:
        problems.append("B 不该在输出（回路断开且波形作废）")

    print("事件流：")
    for ev in events:
        print("  -", ev)
    print("\n状态快照：", state.summary())
    if problems:
        print("\n[FAIL] 自测失败：")
        for p in problems:
            print("  -", p)
        return 1
    print("\n[OK] 自测通过（解析 / 状态 / 事件都对）")
    return 0


def _parse_ch(text: str) -> int:
    t = text.strip().upper()
    if t in ("A", "0"):
        return CH_A
    if t in ("B", "1"):
        return CH_B
    if t in ("AB", "BOTH", "FF"):
        return CH_BOTH
    raise argparse.ArgumentTypeError(f"通道只会是 A / B / AB，得到 {text!r}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="dglab-esp32 上位机示例：实时读取郊狼强度 / 波形 / 回路",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", help="串口名，例如 COM19")
    ap.add_argument("--list", action="store_true", help="列出本机串口")
    ap.add_argument("--selftest", action="store_true", help="不接硬件，自测解析逻辑")
    ap.add_argument("--set", metavar="CH=VAL", action="append", default=[],
                    help="强度绝对值，例如 --set A=20 或 --set AB=0")
    ap.add_argument("--step", metavar="CH=DELTA", action="append", default=[],
                    help="强度相对增减，例如 --step A=+1")
    ap.add_argument("--loop", metavar="CH=on|off", action="append", default=[],
                    help="回路开关，例如 --loop B=off")
    ap.add_argument("--for", dest="duration", type=float, default=0.0,
                    help="跑多少秒后退出（0 = 一直跑，Ctrl+C 停）")
    ap.add_argument("--quiet", action="store_true", help="只打状态行，不打事件")
    args = ap.parse_args(argv)

    if args.list:
        return cmd_list()
    if args.selftest:
        return cmd_selftest()
    if not args.port:
        ap.error("要么 --list / --selftest，要么给一个 --port（例如 --port COM19）")

    def on_event(text: str) -> None:
        if args.quiet:
            return
        sys.stdout.write("\r\x1b[2K" + f"[事件] {text}\n")
        sys.stdout.flush()

    link = DglabLink(args.port, on_event=on_event)
    try:
        link.open()
    except Exception as exc:  # noqa: BLE001
        print(f"打不开 {args.port}：{exc}\n"
              f"（串口是独占的：串口监视器 / webDG / 别的脚本先关掉）", file=sys.stderr)
        return 2

    try:
        # 一次性控制指令
        for item in args.set:
            ch_s, val_s = item.split("=")
            link.set_strength(_parse_ch(ch_s), int(val_s))
            print(f"→ 强度设成 {ch_s.upper()}={val_s}")
        for item in args.step:
            ch_s, val_s = item.split("=")
            link.step_strength(_parse_ch(ch_s), int(val_s))
            print(f"→ 强度增减 {ch_s.upper()} {val_s}")
        for item in args.loop:
            ch_s, val_s = item.split("=")
            link.set_loop(_parse_ch(ch_s), val_s.strip().lower() in ("on", "1", "true", "closed"))
            print(f"→ 回路 {ch_s.upper()} = {val_s}")

        deadline = time.monotonic() + args.duration if args.duration > 0 else None
        print("实时状态（Ctrl+C 退出）：")
        while True:
            sys.stdout.write("\r\x1b[2K" + link.state.summary())
            sys.stdout.flush()
            if deadline is not None and time.monotonic() >= deadline:
                break
            if not link.state.connected:
                break
            time.sleep(0.1)
        print()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
