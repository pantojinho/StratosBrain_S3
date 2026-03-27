# StratosBrain S3

Wearable cockpit and field telemetry OS for the Waveshare ESP32-S3 Touch AMOLED 1.64.

Official project root:

- `C:\Esp32\CLAUDE\StratosBrain_S3`

This project is being built as a single integrated firmware for:

- flight instruments
- meteorological monitoring
- communications and telemetry
- system configuration on-device

The current firmware target is the Waveshare ESP32-S3 Touch AMOLED 1.64 with:

- ESP32-S3R8
- 16 MB Flash
- 8 MB PSRAM
- 1.64 inch AMOLED 280x456
- FT3168 capacitive touch
- QMI8658 IMU
- microSD slot
- Wi-Fi / BLE

## Product direction

The UI is organized around 4 main modes:

1. `PLANE`
2. `METEO`
3. `COMMS`
4. `CONFIG`

Current direction:

- `PLANE` is being simplified into a lighter data-first flight panel
- `METEO` is the local weather station view
- `COMMS` is the network, LoRa and GPS control center
- `CONFIG` is the service, storage and diagnostics area

## Current status

What is already working:

- display initialization on the CO5300 AMOLED
- touch input through FT3168 + LVGL
- main menu and top-level navigation
- integrated Wi-Fi AP mode inside the main firmware
- on-screen Wi-Fi diagnostics such as SSID, password and IP
- green RGB LED signaling for Wi-Fi activity
- microSD blackbox/logging foundation
- BOOT button acting as `Home/Back`
- single-sketch memory optimization through local `lv_conf.h`

What is still in progress:

- real sensor data for altimeter / vertical speed / heading / GPS speed
- final UI polish for wearable readability
- LoRa backend integration
- BLE integration
- weather and skydiving modes with real data

## Repository layout

- `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`
  Main firmware sketch.
- `StratosBrain_LVGL9/lv_conf.h`
  Local LVGL memory/profile tuning used by the firmware.
- `StratosBrain_WebConfig/StratosBrain_WebConfig.ino`
  Legacy Wi-Fi-focused diagnostic/reference sketch kept for comparison only.
- `SESSION_HANDOFF.md`
  Technical project status and next-step notes.
- `_vendor/`
  Vendor demos, references and upstream material kept for reverse-reference only.

## Arduino IDE settings

Use these settings in Arduino IDE:

- Board: `Waveshare ESP32-S3-Touch-AMOLED-1.64`
- USB CDC On Boot: `Enabled`
- Partition Scheme: `16M Flash (3MB APP/9.9MB FATFS)`
- PSRAM: `Enabled`
- Upload Mode: `UART0 / Hardware CDC`
- Upload Speed: `921600`

## Build notes

This project is designed to run as a single firmware, not as separate test sketches.

Important details:

- Wi-Fi AP is integrated into the main firmware
- when Wi-Fi is active, the runtime enters a lighter mode to reduce pressure on the device
- the local `lv_conf.h` is required to avoid LVGL's default static RAM footprint

## License

This project is licensed under the MIT License.

See [LICENSE](LICENSE).
