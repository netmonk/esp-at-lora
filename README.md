# LoRa AT Modem — TTGO LoRa32 T3 v1.6.1 (433 MHz)

> 🇫🇷 [Version française](README.fr.md)

PlatformIO/Arduino firmware that turns a **LilyGO TTGO LoRa32 T3 v1.6.1**
board (ESP32 + SX1278, 433 MHz) into a raw LoRa modem driven by **AT**
commands over the USB serial port.

The implemented command set follows `AT_COMMANDS.md` (at-os3 interface):
frequency, modulation profile (SF/BW/CR/preamble), packet profile
(CRC/LDRO/length), PA_BOOST output power, sync word, IQ inversion,
hex-encoded frame transmit/receive, SX1278 register read, etc.

The radio driver is **[RadioLib](https://github.com/jgromes/RadioLib)**.

## Pinout (T3 v1.6.1)

LoRa SX1278 (SPI):

| Signal | GPIO |
|--------|-----:|
| SCK    | 5    |
| MISO   | 19   |
| MOSI   | 27   |
| NSS/CS | 18   |
| RST    | 23   |
| DIO0   | 26   |
| DIO1   | 33   |

OLED SSD1306 128×64 (I²C, address 0x3C):

| Signal | GPIO |
|--------|-----:|
| SDA    | 21   |
| SCL    | 22   |
| RST    | -1 (not wired) |

## OLED display — indicators

The screen shows in real time (~3 Hz):

```
┌──────────────────────────┐
│ 433.500MHz           RX   │  frequency + mode (RX / STBY)
│ SF9 BW125 4/5            │  modulation profile
│ P10 SW12 IQn C1         │  power, sync word, IQ, CRC
│ TX:1 RX:0 E:0           │  counters (TX / RX / CRC errors)
│ RSSI-57 SNR7            │  last received packet metrics
│ TX 5B 68656C6C6F        │  last event + payload (truncated hex)
└──────────────────────────┘
```

The OLED is **optional**: if no display is detected, the AT modem keeps
working normally. If the screen stays black, some board revisions wire the
OLED reset to GPIO16 — rebuild with `-D PIN_OLED_RST=16` in
`platformio.ini`.

These values are passed as `build_flags` in `platformio.ini`; adjust them
if your board revision differs.

## Build / flash

```bash
pio run                # build
pio run -t upload      # flash over USB
pio device monitor -b 115200   # serial console
```

## Serial link

- **115200 baud, 8N1**
- `\n` executes a command; `\r` is ignored (so `\r\n` is accepted)
- Uppercase commands, strict parsing
- Responses terminated by `\r\n`
- Unknown / malformed command → `+ERR=4`
- On boot: `+READY`

## Example — receive

```text
AT
AT+BAND=436995000
AT+PARAMETER=8,6,3,8
AT+PKT=1,2,0
AT+SYNCWORD=18
AT+IQI=0
AT+CRFOP=5
AT+MODE=0
```

When a packet is received:

```text
+RCV=<addr>,<len>,<hexdata>,<rssi_dbm>,<snr_db>,<freq_err_hz>
```

## Example — transmit

```text
AT+SEND=0,4,70696E67     # sends "ping"
+OK
+SENDED
```

## Implementation notes

- `AT+ADDRESS` and `AT+NETWORKID` are compatibility "shadow" registers:
  they do not alter the contents of raw LoRa packets (the address only
  appears as the first field of `+RCV`).
- `AT+IPR` is a compatibility no-op: the serial baud rate is not changed.
- `AT+REG=<hh>` reads an SX1278 register through a RadioLib subclass
  (`SX1278AT`) because `getMod()` is protected in RadioLib 6.x.
- RSSI/SNR/FEI come from `getRSSI()`, `getSNR()`, `getFrequencyError()`.
- ⚠️ The power and frequency values accepted by the firmware do not
  guarantee regulatory compliance for transmission in your region/band.

## License

Copyright (C) 2026 plonky

This program is free software, distributed under the terms of the
**GNU General Public License version 3** (or any later version); see the
[LICENSE](LICENSE) file. It comes with absolutely no warranty.
