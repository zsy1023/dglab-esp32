# dglab-esp32

**An ESP32 re-implementation of the DG-LAB Coyote 3.0 BLE peripheral.**

It makes the official DG-LAB app on your phone believe it is talking to a real Coyote host,
and hands every byte the app writes over a single USB serial link to your own PC-side host
program. Strength replies coming back from that host are notified to the app on the same path.

```
Controller phone, DG-LAB app ──BLE──▶ ESP32 (this firmware) ──serial 115200──▶ PC-side host
                                        ▲                                        │
                                        └────────── B1 strength reply ───────────┘
```

> 中文版：**[README.md](README.md)**

---

## What it does / does not do

**Does** (its only job is to faithfully impersonate one host):

- Reproduces the full real-device GATT table and the advertising name `47L121000`
  (including characteristics that are absent from the official docs)
- Reproduces real-device behaviour: sends the 6-byte `53 00 <last 4 MAC bytes>` handshake
  **immediately** after `0x150B` is subscribed
- Forwards the app's `B0`/`BF` commands **verbatim** over serial (no parsing, no reassembly)
- Notifies `0x150B` when the host sends a `B1` reply
- Restarts advertising automatically after a disconnect

**Does not do** (that is the PC-side host's job):

- Protocol semantics, waveform stacking, punishment logic
- Networking / relaying / multi-device orchestration

> In other words: this firmware is a **transparent pipe**. Whatever the app wrote is handed to
> the host program byte for byte.

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
| `0xB0` + 19B | **20** | The app's `B0` command, passed through **verbatim** |
| `0xBF` + 6B | **7** | The app's `BF` command, passed through **verbatim** |
| `0xCC` | **1** | The app connected |
| `0xCD` | **1** | The app disconnected |
| Text lines starting with `#` | variable | Human-readable log (discard line by line, see [§4.2](#42-log-convention-coexisting-with-binary-frames)) |

### 2.3 Downlink: host program → ESP32

| Frame | Length | Meaning |
|---|---|---|
| `0xB1` + 3B | **4** | Strength reply → the firmware notifies it on `0x150B` |
| `0xCE` | **1** | Heartbeat → the firmware answers with one `#` log line ("is the other end really this firmware?") |

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
#dglab-esp32 up: name=47L121000 bt_mac=EC:E3:34:A9:13:0E
#App connected (conn_id=0)
#0x150B subscribed -> sending device handshake 53 00 34 A9 13 0E
#B1 <- host: B1 01 0A 00
#stats up=736 drops=8 down_bytes=210 b1=52 unknown=0 conn=1 sub=1
```

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

One "app changes strength → UI echoes it" round trip passes through:
`BLE round trip` + `serial uplink` + `host parsing/reply` + `serial downlink` + `BLE notification`.

Measured on our machine:

| Segment | Time |
|---|---|
| Serial loopback (host writes → firmware handles → host receives), **after tuning** | **p50 4.5 ms** |
| Same, before tuning (one blocking `read(256)` on the host) | p50 87.9 ms ← what used to feel "laggy" |

What the firmware does for this: `loop()` polls every 1 ms, requests 15–30 ms connection
parameters after connect, and emits a status log line only every 8 s (to keep the UART free).

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
3. **The app also writes non-`B0`/`BF` data to `0x150A`** (measured roughly once every 20 s, shaped
   like `BC 04 32 32 32 1E 50 96 32`). It is dropped per the contract and logged. The meaning is
   unknown and it does not affect functionality for now.
4. **Subscribing to `0x1500`/`0x2A59` on an unencrypted link**: the real device is rejected on
   Windows too (`Access Denied`); it is a link/host-side limitation and does not affect the phone app.

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
