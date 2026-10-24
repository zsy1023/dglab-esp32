# dglab-esp32

**An ESP32 re-implementation of the DG-LAB Coyote 3.0 BLE peripheral — and the board *is* the host.**

It makes the official DG-LAB app on your phone believe it is talking to a real Coyote host:
`B0`/`BF` commands are parsed **by this firmware**, channel strengths / soft caps / waveforms are
maintained here, and `B1` replies are sent here — so **the phone alone can change strength without
any PC attached**. The same raw commands are still forwarded over serial to your PC-side program
(which is now a mere *observer*).

```
Controller phone, DG-LAB app ──BLE──▶ ESP32 (this firmware = host, answers B1 itself)
                                          │
                                          └──serial 115200──▶ PC-side program (raw frames + '#' logs, no B1)
```

> 中文版：**[README.md](README.md)**

---

## What it does / does not do

**Does** (faithfully impersonating one host):

- Reproduces the full real-device GATT table and the advertising name `47L121000`
  (including characteristics that are absent from the official docs)
- Reproduces real-device behaviour: sends the 6-byte `53 00 <last 4 MAC bytes>` handshake
  **immediately** after `0x150B` is subscribed
- **★ Host logic**: `B0` → per-channel strength (absolute / relative / clamped by soft cap) and
  two channels × 4 waveform groups; `BF` → strength soft caps + balance parameters (soft caps are
  **persisted across power loss**); replies `B1` whenever sequence number ≠ 0
- Forwards `B0`/`BF` verbatim over serial for the PC side (off by default; turned on by the PC's
  `0xCE` heartbeat, or manually with `0xC1`)
- Pure-ASCII serial logs print **strength / waveform / soft caps** right in a serial monitor
- Restarts advertising automatically after a disconnect

**Does not do** (that is the PC-side program's job):

- Waveform stacking, punishment logic, rule engine
- Networking / relaying / multi-device orchestration

> In other words: this firmware = **a real host + a transparent side-channel**.

---

## 1. Hardware & wiring

| Item | Value |
|---|---|
| Board | Any ESP32 dev board (this project was tested on an **ESP32-D0WD-V3**, BLE 4.2, onboard CH340 USB-serial) |
| Wiring | **One USB data cable** to the PC (power + flashing + data). The BLE antenna is onboard, no extra hardware |
| Phone | Official DG-LAB app, within ~1–10 m of the board (the app talks BLE; the serial link is unrelated) |

---

## 2. Ports & interfaces (the upstream link)

### 2.1 Serial link to the host program — this is the "upstream"

| Item | Value |
|---|---|
| Physical interface | Onboard USB-serial (WCH CH340) |
| **Serial parameters** | **115200 8N1, no hardware flow control** |
| **How to identify it** | `USB VID:PID = 1A86:7523` |
| Port name | `COM18` on the machine we tested (changes per machine and per USB port) |
| Purpose | Power + flashing + **the data link to the host program** (the same port) |

> ⚠️ **Do not pick a Windows Bluetooth virtual COM port** (commonly `COM5`/`COM6`). It **opens
> successfully but never delivers a single byte** — it looks fine and simply has no data.
> Pick the one with `VID:PID = 1A86:7523`.
>
> ⚠️ The serial port is **exclusive**: stop the host program before flashing or monitoring,
> otherwise the port will not open, or the two will fight over bytes.

### 2.2 Uplink: ESP32 → host program

| Frame | Length | Meaning |
|---|---|---|
| `0xB0` + 19B | **20** | The app's `B0` command, passed through **verbatim** (off by default, see below) |
| `0xBF` + 6B | **7** | The app's `BF` command, passed through **verbatim** (off by default, see below) |
| `0xCC` | **1** | The app connected |
| `0xCD` | **1** | The app disconnected |
| Text lines starting with `#` | variable | Human-readable log (discard line by line, see [§4.2](#42-log-convention-coexisting-with-binary-frames)) |

> **Raw frames are off by default**: they start flowing only after the PC program sends a `0xCE`
> heartbeat (its startup probe does that already, so **no PC-side code change is required**).
> That keeps a serial monitor free of binary garbage when only the phone is connected.
> Toggle manually with `0xC1` (on) / `0xC0` (off); `#STAT` shows the current state as `raw=`.

### 2.3 Downlink: host program → ESP32

| Frame | Length | Meaning |
|---|---|---|
| `0xCE` | **1** | Heartbeat → the firmware answers with one `#` log line and **turns raw uplink on** |
| `0xC0` / `0xC1` | **1** | **Disable / enable** raw uplink. With `0xC0` the PC program stops receiving `B0`/`BF` — debug only |
| `0xC2` / `0xC3` | **1** | **Disable / enable** per-`B0` dumping (**enabled by default**). Each `B0` prints one line — 100 ms cadence, which is the app's write rate; send `0xC2` if it is too chatty |
| `0xD0` + 2B | **3** | **Absolute strength** `0xD0 ch value` (0–200, clamped further by the soft cap) |
| `0xD1` + 2B | **3** | **Relative strength** `0xD1 ch delta`, delta as int8 (`0xFF` = −1, `0x01` = +1) |
| `0xD2` + 2B | **3** | **Loop switch** `0xD2 ch on`: `0` = open (no current on real hardware) / non-zero = closed |
| `0xB1` + 3B | **4** | **Deprecated, ignored** (one warning is logged). The firmware owns the host logic now — stop sending B1 |

> `ch` is **`0` = channel A, `1` = channel B, `0xFF` = both**. After a strength change the firmware
> answers with a **`B1(seq=0)`** (exactly what a real device does when you turn its wheel), so the
> phone app's display stays in sync.

> These control bytes exist so you can toggle things by hand from a serial monitor.

### 2.4 BLE side: phone app → ESP32

| Item | Value |
|---|---|
| Advertising / device name | `47L121000` |
| Advertising payload | **Name only** — no service UUID, no manufacturer data |
| Connection interval | The firmware requests 15–30 ms after connect (the app, as central, makes the final call) |
| MTU | 23 (measured on the real device; the 20-byte `B0` fits exactly) |
| Services / characteristics | See [§5 GATT table](#5-reproduced-gatt-table) |

---

## 3. Build / flash / watch the log

You need [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).

```bash
pio run                                      # build
pio run -t upload --upload-port COM18        # flash (replace COM18 with your port)
pio device monitor -p COM18 -b 115200        # watch the '#' log (optional)
```

The platform and framework versions are pinned in `platformio.ini`
(`espressif32@6.4.0` + `framework-arduinoespressif32@3.20011`); PlatformIO downloads them on
the first build (network required).

### 3.1 Serial scope (web page, recommended)

`tools/serial_scope.html` — a **single-file page** that talks to the board's COM port through the
browser's Web Serial API and turns the firmware's `#` lines into a live UI (and can control it):

- **See**: per-channel strength (current / soft cap), the waveform as a **scrolling area chart +
  frequency trace** (each point = one 25 ms group, 10 s window), loop state, B0 frame rate,
  whether the phone app is connected/subscribed, and the last `B1` reply
- **The waveform area says whether it is actually outputting**: `输出中 (peak/freq)` /
  `波形强度全 0` / `整组作废 (which group is out of range)` / `回路断开` / `还没收到 B0`
- **Control**: per-channel **loop switches** (`0xD2`), strength **−/＋ nudge** (hold to repeat,
  `0xD1`), and **absolute strength** (`0xD0`). The firmware answers each change with `B1(seq=0)`,
  so the phone app follows along

Usage: close any serial monitor / PC program first (the port is exclusive), then open the file in
Chrome or Edge (if `file://` is blocked: `python -m http.server 8000` in that folder and open
`http://localhost:8000/serial_scope.html`), pick the port (115200 8N1). The board may reset once
when the port opens (DTR/RTS glitch) — that is normal.

> The page never sends `0xCE` (that byte makes the firmware start pushing raw binary frames).

### 3.2 Python host example (read strength in real time)

`examples/host_example.py` — copy this when writing your own host program. Only needs `pyserial`:

```bash
pip install -r examples/requirements.txt

python examples/host_example.py --list                  # list serial ports
python examples/host_example.py --port COM19            # live strength / wave / loop
python examples/host_example.py --port COM19 --set A=20 --loop B=off --for 5
python examples/host_example.py --selftest              # no hardware needed
```

"Real time" comes from the firmware's **one `#B0` line every 100 ms** (4 groups × 25 ms per frame):

```
A  20/150 [闭环] wA=20:40,20:50,20:60,20:70 峰 70 | B   0/30  [断开] wB=!0:0,0:0,0:0,0:101 | app=连上/已订阅 b0=10.0/s
```

As a library:

```python
from examples.host_example import DglabLink, CH_A, CH_B

link = DglabLink("COM19", on_event=lambda ev: print("[event]", ev))
link.open()                       # sends 0xC3 (verbose logs) + 0xC0 (no raw binary frames)

print(link.state.a.strength)      # current strength, refreshed every 100 ms
print(link.state.a.cap)           # soft cap
print(link.state.a.wave)          # [(freq, amp) x 4]
print(link.state.a.outputting)    # actually outputting? (loop + valid wave + peak > 0)
print(link.state.b0_rate)         # measured frame rate (10.0 when healthy)

link.set_strength(CH_A, 20)       # 0xD0 absolute (clamped by the soft cap)
link.step_strength(CH_B, +1)      # 0xD1 relative
link.set_loop(CH_B, False)        # 0xD2 open the loop
link.close()
```

Strength changes made here are **mirrored to the phone app**: the firmware answers `B1(seq=0)`.

Tests with a fake serial port (no board needed): `python examples/test_host_example.py`.

---

## 4. Serial frame protocol (the contract)

Firmware-side constants live at the top of `src/main.cpp`; implement your host program to match.
**Changing this requires changing both sides.**

### 4.1 Framing

**Fixed-length framing keyed on the HEAD byte** — no length prefix, no checksum.

- Writes whose length/head byte do not match the contract **never enter the link** (so they
  cannot corrupt the fixed-length framing), but they are logged.
- Fixed baud rate **115200 8N1** (both sides must agree).

### 4.2 Log convention: coexisting with binary frames

Every human-readable log line from the firmware starts with **`#`** and ends with **`\n`**;
the host program discards whole lines while framing:

```
#dglab-esp32 boot: name=47L121000 bt_mac=EC:E3:34:A9:13:0E
#caps: host_logic=1 (firmware owns strength+B1) soft_cap=150/30 raw_uplink=0 verbose_b0=1
#App connected (conn_id=0) | strength A=0/150 B=0/30
#0x150B subscribed -> device handshake 53 00 34 A9 13 0E
#B0 t=1520 seq=0 mode=A./B. set=0/0 A=0/150 B=0/30 link=A1/B1 wA=10:0,10:10,20:20,30:30 wB=!0:0,0:0,0:0,0:101 raw=B00000000A0A0A0A000A141E0000000000000065
#B1 -> App seq=1 A=5 B=0
#STRENGTH A=5/150 B=0/30 (seq=1)
#STAT t=2620 A=5/150 B=0/30 link=A1/B1 wA=10:0,10:10,20:20,30:30 wB=!0:0,0:0,0:0,0:101 | b0=12 (10.0/s) bf=1 b1=1 drop=0 down=0 logdrop=0 conn=1 sub=1 raw=0 verbose=1
```

You can read **strength** (`A=5/150` = current/soft cap), **waveform** (`wA=` = four `freq:amp`
groups, comma-separated so it stays one token), **loop state** (`link=A1/B1`, 1 = closed / 0 = open)
and **soft caps** (the `SOFT LIMIT` line).

> A **`!` prefix means the whole 4-group channel is discarded** (a real device outputs nothing).
> The numbers are still printed (`wB=!0:0,0:0,0:0,0:101` → freq 0, amp 101) so "why is there no
> waveform?" is answerable at a glance; `raw=` at the end is the **raw 20-byte `B0` frame**.

Logs are **pure ASCII on purpose**: earlier versions printed UTF-8 Chinese, which turns into
mojibake in a GBK-decoding serial monitor.

**Refresh rate**: `#B0` prints for every frame by default, following the app's write cadence —
**one frame every 100 ms, 4 groups of 25 ms per channel**, the finest granularity the V3 protocol
offers. The `t=` field is `millis()`, so you can verify the cadence (should tick every 100 ms).
`#STAT` also reports `b0=12 (10.0/s)` so dropped frames are obvious. Send `0xC2` to silence it
(`0xC3` re-enables); when the TX buffer is full the firmware **drops log lines instead of
blocking** and counts them in `logdrop=`.

`#` (`0x23`) is **not in the set of head bytes**, so the two can never be confused; a `0x0A`
inside a frame body is never treated as a newline either (lines are only consumed while waiting
for a head byte). This convention lets **logs** and **protocol** share UART0 without interfering.

> ⚠️ When using this firmware, **never print text without `#` on the same serial port**.

### 4.3 Fault handling & protection

Implemented in the firmware:

- **Half-frame watchdog**: if a frame is not completed within 200 ms after its head byte,
  it is dropped and the parser resynchronises (otherwise one lost byte permanently wedges the downlink).
- **Subscribe edge detection**: polls the `BLE2902` (CCCD) local value and sends the device
  handshake on the rising edge (Bluedroid updates it when the central writes the CCCD).
- **Liveness answer**: replies with one `#` log line when `0xCE` is received.

Recommended for the host program:

- **Uplink buffer fallback**: if the buffer exceeds 4096 bytes, clear it and resynchronise.
- **Liveness probe**: send `0xCE` right after opening the port and **fail loudly** if no frame or
  log line arrives ("opened fine but the other end is silent" = wrong port / firmware not flashed).
  Do not hang silently.
- **Reconnect**: handle unplug / reset / port takeover with backoff retries.

### 4.4 Host logic (the firmware computes strength itself)

The state machine lives in `src/host_state.h` (**pure C++ — not a single Arduino/BLE line**, so it
can be unit-tested with plain `g++`). Semantics follow the official V3 spec + real-device measurements:

| Rule | Implementation |
|---|---|
| `B0` strength mode (byte1 low nibble) | `00` unchanged / `01` relative add / `10` relative subtract / `11` absolute set |
| Setpoint out of range (>200) | treated as **0** (not clamped to 200 — the official doc's e.g. 4 says so) |
| Absolute strength | clamped to `0 ~ soft cap`; the cap comes from `BF` and is **persisted across power loss** (NVS) |
| Waveform validity | frequency `10~240`, amplitude `0~100`; **any invalid value discards the whole 4-group channel** |
| When `B1` is sent | **whenever the sequence number ≠ 0** (measured: even if strength did not change) |
| `BF` soft cap >200 | not applied (official doc: out-of-range values do not modify) |

> Why `B1` matters: after changing strength the app **locks its input** until it receives a `B1`
> with the same sequence number. No `B1` = strength never moves on the phone and stays 0 on screen.

**Changes required on the PC side:**

1. **Stop writing `B1`** — the firmware is the host now and answers itself; incoming `B1` is ignored
   with a warning.
2. To display strength, keep parsing uplink `B0` yourself (same rules), or read the `#STRENGTH` /
   `#STAT` log lines.
3. `0xCE` heartbeat and the `B0`/`BF`/`CC`/`CD` uplink are unchanged — old code keeps working
   (the heartbeat automatically enables raw uplink). Just drop the `B1` write.
4. The serial port is **exclusive**: a serial monitor holding it blocks the PC program, and vice versa.

---

## 5. Reproduced GATT table

| Service | Characteristic | Properties | Value / notes |
|---|---|---|---|
| `0x1800` (built into the stack) | `0x2A00` Device Name | read, write | `47L121000` |
| | `0x2A01` / `0x2AA6` | read | Appearance / address resolution |
| `0x1801` (built into the stack) | `0x2A05` | indicate | Service Changed |
| `0x180C` | `0x150A` | **write-without-response** | Every app command (**not a plain WRITE!**) |
| | `0x150B` | notify | Every reply from this firmware (`B1`, handshake, …) |
| `0x180A` | `0x1500` | read, notify | Battery, 1 byte percent (`0x64` = 100%) |
| | `0x1501` | read | `0c 03` — **the app decides the protocol is V3 from this; you must return it** |
| | `0x1502` | read | 6-byte local MAC |
| | `0x2A25` | read | `"CEB3%Q"` + MAC (12 bytes) |
| | `0x2A59` | read, notify | 10-byte output status readback (see [§7](#7-known-issues--todo) item 1) |
| `0x2003` | `0x0007` / `0x0008` | write / read+notify | Purpose unknown, provided to match the real device |
| `0x2004` | `0x0009` | read, write | Same as above |
| `0xFE59` | Nordic DFU | — | **Not reproduced** (present on the real device, not needed by the app) |

Behaviour details (all measured on a real device, not copied from the official docs):

- Advertising contains the **name only** — no service UUID, no manufacturer data.
- Right after `0x150B` is subscribed, 6 bytes `53 00 <last 4 MAC bytes>` are sent.
- Advertising restarts **automatically** after a disconnect (otherwise the phone can never find it again).

App initialisation order (reproduced as-is):

```
read(180A/1500) -> read(180A/1501) -> notify(180A/1500)
   -> notify(180C/150B) -> check(180C/150A) -> notify(180A/2A59)
```

---

## 6. Latency

One "app changes strength → UI echoes it" round trip used to pass through:
`BLE round trip` + `serial uplink` + `host parsing/reply` + `serial downlink` + `BLE notification`.

It now **skips the PC entirely** (the firmware answers `B1` itself): just
`BLE write` + `host logic + B1 queue` + `notify` from `loop()` (1 ms cycle).

Measured on our machine (serial path, for reference):

| Segment | Time |
|---|---|
| Serial loopback (host writes → firmware handles → host receives), **after tuning** | **p50 4.5 ms** |
| Same, before tuning (one blocking `read(256)` on the host) | p50 87.9 ms ← what used to feel "laggy" |

What the firmware does for this: `loop()` polls every 1 ms, requests 15–30 ms connection
parameters after connect, and prints `#STAT` once a second while connected (every 5 s otherwise).

### 6.1 Hardware-free unit test (run it whenever you touch the state machine)

`src/host_state.h` has no Arduino dependency, so any g++ can build the test:

```bash
# inside firmware/
g++ -std=c++11 -Wall -Wextra -I src -o test_host_state tools/test_host_state.cpp && ./test_host_state
```

It covers: absolute/relative strength, out-of-range setpoints treated as 0, soft-cap clamping,
clamping when a soft cap is lowered, `B1` required whenever `seq≠0`, entire waveform group
discarded on any invalid value, bad length/HEAD leaving state untouched, and a real packet from
the official `example.md`.

---

## 7. Known issues / TODO

1. **`0x2A59` is an approximation**: the exact encoding rule of the real device is still unknown
   (we only know it changes with the waveform in `B0`). The current implementation keeps bytes 0–1
   at `01 01`, tracks the A/B channel waveform strengths in bytes 2/3, and leaves the rest at the
   baseline. The app subscribes to it and works fine so far; an exact reproduction needs more
   measurements on real hardware.
2. **`0x2A04` (preferred connection parameters) is missing from `0x1800`**: that characteristic is
   provided by the ESP-IDF stack based on build configuration, and the precompiled libraries of the
   current Arduino core do not enable it — adding it means rebuilding IDF. Instead, the firmware
   actively requests fast connection parameters after connect for the same effect.
3. **The app also writes non-host commands to `0x150A`** (a few per connection: `50 02 00 00 …`
   (civet), `BC 04 32 32 32 1E 50 96 32`, `C3 06 …`, `C4 0A`, `FF`). Those are command heads of
   other DG-LAB devices (civet / paw-prints); they are ignored per the contract and logged as
   `#IGNORED ...` without polluting the frame stream. No functional impact observed.
4. **Subscribing to `0x1500`/`0x2A59` on an unencrypted link**: the real device is rejected on
   Windows too (`Access Denied`); it is a link/host-side limitation and does not affect the phone app.
5. **`B1` is only sent when the sequence number ≠ 0** (matching real-device measurements). A real
   device also sends `seq=0` `B1` when its physical wheel changes strength; this firmware has no
   wheel, so that never happens. Strength changes commanded by the app are fully visible.

---

## 8. License

Copyright (C) 2026 zsyfpv

Released under the **GNU General Public License v3.0** (or later); the full text is in
[LICENSE](LICENSE).

```
dglab-esp32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

dglab-esp32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
```

---

## Disclaimer

- This is an **unofficial** third-party firmware with **no affiliation or authorisation** from
  DG-LAB / Coyote.
- Names and trademarks belong to their respective owners. This project contains no official
  firmware, binaries, or reverse-engineered artefacts; all protocol behaviour comes from
  **interoperability observations** of the public BLE interface.
- For learning, interoperability research, and experiments on your own hardware only.
  **Use it at your own risk** — where electrical stimulation devices are involved, follow local
  regulations and treat personal safety with the utmost care.
