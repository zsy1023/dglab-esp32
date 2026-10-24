#!/usr/bin/env python3
"""host_example.py 的集成测试：**不需要板子**（用一个假的串口对象顶替 pyserial）。

跑法：

    python examples/test_host_example.py

它验证三件事：

1. ``DglabLink.open()`` 打开后自动发的两个字节对不对（``0xC3`` 开逐帧日志、``0xC0`` 关原始帧）
2. 喂固件真实格式的日志行 → ``state`` 里的强度 / 软上限 / 回路 / 波形 / 帧率 / B1 都对
3. 三个控制方法发出的字节对不对（``0xD0`` / ``0xD1`` / ``0xD2``，通道与补码算法）
"""

from __future__ import annotations

import queue
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import host_example as H  # noqa: E402


class FakeSerial:
    """冒充 pyserial.Serial：只会"吐"预置的行，并记录写出去的字节。"""

    #: 假串口收到的字节流（每条一次 readline）
    inbox: "queue.Queue[bytes]" = queue.Queue()

    def __init__(self) -> None:
        self.written: list[bytes] = []
        self.dtr = True
        self.rts = True
        self.timeout = None
        self.write_timeout = None
        self.is_open = False

    # DglabLink 只用到这几个成员
    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        while not self.inbox.empty():
            self.inbox.get_nowait()

    def readline(self) -> bytes:
        try:
            return self.inbox.get_nowait()
        except queue.Empty:
            time.sleep(0.005)   # 别忙转：让读线程真的"等一会儿"
            return b""

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)


class HostExampleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fake = FakeSerial()
        self._real_serial = H.serial.Serial
        H.serial.Serial = lambda: self.fake          # type: ignore[assignment]
        self.events: list[str] = []

    def tearDown(self) -> None:
        H.serial.Serial = self._real_serial          # type: ignore[assignment]

    def _feed(self, *lines: str) -> None:
        for line in lines:
            self.fake.inbox.put((line + "\n").encode("utf-8"))

    def _wait_for(self, predicate, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.01)
        return predicate()

    def test_opens_link_and_asks_for_text_logging(self) -> None:
        link = H.DglabLink("COM_TEST", on_event=self.events.append)
        link.open()
        try:
            self.assertTrue(link.state.connected)
            # 打开后先开逐帧日志（0xC3），再关原始二进制帧（0xC0）
            self.assertEqual(self.fake.written, [b"\xc3", b"\xc0"],
                             f"打开时发的字节不对：{self.fake.written}")
            # DTR/RTS 必须被压住，否则板子会被复位
            self.assertFalse(self.fake.dtr)
            self.assertFalse(self.fake.rts)
        finally:
            link.close()
        self.assertFalse(link.state.connected)

    def test_realtime_state_from_firmware_lines(self) -> None:
        link = H.DglabLink("COM_TEST", on_event=self.events.append)
        link.open()
        try:
            self._feed(
                "#App connected (conn_id=0) | strength A=0/150 B=0/30",
                "#B0 t=1520 seq=0 mode=A./B. set=0/0 A=5/150 B=0/30 link=A1/B1 "
                "wA=20:40,20:50,20:60,20:70 wB=!0:0,0:0,0:0,0:101 "
                "raw=B00005001E14202832000000000000000065",
                "#STAT t=2620 A=5/150 B=0/30 link=A1/B1 wA=20:40,20:50,20:60,20:70 "
                "wB=!0:0,0:0,0:0,0:101 | b0=12 (10.0/s) bf=1 b1=1 drop=0 down=0 "
                "logdrop=0 conn=1 sub=1 raw=0 verbose=1",
                "#B1 -> App seq=3 A=7 B=0",
                "#STRENGTH A=7/150 B=0/30 (via serial cmd 0xD1)",
                "#link: loop A=closed B=open | strength A=7/150 B=0/30",
            )
            ok = self._wait_for(lambda: link.state.a.strength == 7 and link.state.b1_count == 1)
            self.assertTrue(ok, f"没等到状态更新：{link.state}")

            st = link.state
            self.assertEqual((st.a.strength, st.a.cap), (7, 150))
            self.assertEqual((st.b.strength, st.b.cap), (0, 30))
            self.assertTrue(st.a.loop)
            self.assertFalse(st.b.loop)             # 最后一条 link: loop 把 B 断开了
            self.assertEqual(st.a.wave, [(20, 40), (20, 50), (20, 60), (20, 70)])
            self.assertTrue(st.a.wave_valid)
            self.assertFalse(st.b.wave_valid)       # '!' = 整组作废
            self.assertEqual(st.b.wave, [(0, 0), (0, 0), (0, 0), (0, 101)])
            self.assertTrue(st.app_connected)
            self.assertTrue(st.app_subscribed)
            self.assertAlmostEqual(st.b0_rate, 10.0)
            self.assertEqual(st.last_b1, (3, 7, 0))
            # A：回路闭合 + 波形合法 + 峰值 70 → 在输出；B：回路断开 → 不输出
            self.assertTrue(st.a.outputting)
            self.assertFalse(st.b.outputting)
            # 事件流里应该看到 B1 / 强度变化 / 回路
            self._wait_for(lambda: len(self.events) >= 3)
            self.assertTrue(any("B1" in e for e in self.events), self.events)
            self.assertTrue(any("强度变化" in e for e in self.events), self.events)
            self.assertTrue(any("回路" in e for e in self.events), self.events)
        finally:
            link.close()

    def test_control_frames(self) -> None:
        link = H.DglabLink("COM_TEST")
        link.open()
        try:
            self.fake.written.clear()
            link.set_strength(H.CH_A, 20)
            link.set_strength(H.CH_BOTH, 300)       # 超范围要夹到 200
            link.step_strength(H.CH_B, +1)
            link.step_strength(H.CH_B, -1)          # -1 → 0xFF（int8 补码）
            link.set_loop(H.CH_B, False)
            link.set_loop(H.CH_A, True)
            link.send_verbose(False)
            link.send_raw(True)
            self.assertEqual(
                self.fake.written,
                [
                    bytes([0xD0, 0x00, 20]),        # 强度绝对值 A=20
                    bytes([0xD0, 0xFF, 200]),       # 两个都改，300 → 200
                    bytes([0xD1, 0x01, 0x01]),      # B +1
                    bytes([0xD1, 0x01, 0xFF]),      # B -1
                    bytes([0xD2, 0x01, 0x00]),      # B 回路断开
                    bytes([0xD2, 0x00, 0x01]),      # A 回路闭合
                    bytes([0xC2]),                  # 关逐帧日志
                    bytes([0xC1]),                  # 开原始帧
                ],
                self.fake.written,
            )
        finally:
            link.close()

    def test_wave_cell_parser(self) -> None:
        self.assertEqual(H.parse_wave_cell("10:0,10:5,20:10,30:50"),
                         (True, [(10, 0), (10, 5), (20, 10), (30, 50)]))
        self.assertEqual(H.parse_wave_cell("!0:0,0:0,0:0,0:101"),
                         (False, [(0, 0), (0, 0), (0, 0), (0, 101)]))
        self.assertEqual(H.parse_wave_cell(""), (False, []))
        self.assertEqual(H.parse_wave_cell("10:0,10:5"), (False, []))  # 组数不对 → 当没数据
        self.assertEqual(H.parse_wave_cell("10:0,xx:1,20:2,30:3"), (False, []))


if __name__ == "__main__":
    unittest.main(verbosity=2)
