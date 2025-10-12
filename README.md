# epd_WUXI_VISION

Reproducible PlatformIO project for Raspberry Pi Pico (RP2040) driving a tri‑color e‑paper via GxEPD2.

## What’s included
- Board/core: Raspberry Pi Pico (Earle Philhower Arduino-Pico core)
- Platform version pinned: raspberrypi@1.17.0
- Libraries pinned:
  - GxEPD2 @ 1.6.0
  - Adafruit GFX @ 1.11.11
- Upload protocol: picotool (auto BOOTSEL)

See `platformio.ini` for exact versions.

## Build and Upload
- Prereqs: Install VS Code + PlatformIO IDE (or PlatformIO Core CLI)
- Connect Pico via USB.
- Build:
```
pio run
```
- Upload (auto BOOTSEL reset supported):
```
pio run -t upload
```
- Serial monitor (115200 baud):
```
pio device monitor -b 115200
```

## Pins (default)
- SCK: GP2
- MOSI: GP3
- MISO: GP4
- CS: GP1
- DC: GP6
- RST: GP7
- BUSY: GP8

## If upload fails
- Hold BOOTSEL while plugging in Pico once, then re-run upload.
- On Linux, set `upload_port = /dev/ttyACM0` in `platformio.ini` if auto-detect fails.
- On Windows, ensure USB driver is OK; picotool comes with the core toolchain.

## If a colleague cannot compile
- Ensure they use PlatformIO and do NOT install libraries manually into Arduino IDE.
- The versions are pinned in `platformio.ini`; run:
```
pio run -t clean
pio pkg update
pio run
```
- If there are still issues, delete `.pio` folder and retry.

## Notes
- First refresh on tri‑color panels can take longer (>10s).
- Serial commands are available; run `h` in the serial monitor for help.
