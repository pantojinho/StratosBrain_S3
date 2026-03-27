# StratosBrain S3 - Session Handoff

**Date:** 2026-03-27  
**Status:** Hardware base OK, but UI/navigation needs redesign for wearable use before adding more features

---

## Official root

This repository is now the official project root:

- `C:\Esp32\CLAUDE\StratosBrain_S3`

## Hardware

| Component | Details |
|---|---|
| Board | Waveshare ESP32-S3-Touch-AMOLED-1.64 |
| MCU | ESP32-S3R8, 16MB Flash, 8MB OPI PSRAM |
| Display | CO5300 AMOLED, 280x456, QSPI |
| Touch | FT3168, I2C `0x38` |
| IMU onboard | QMI8658, I2C `0x6B` or `0x6A` |
| Sensor HAT | BME688 `0x77`, BMP581 `0x47`, BMM350 `0x14`, LTR-390UV `0x53`, MAX17048 `0x36` |
| GPS | AT6558R, UART TX=`43` RX=`44`, 9600 baud |
| LoRa | CC68 915MHz, SPI CS=`7` SCK=`15` MISO=`16` MOSI=`17` |
| SD Card | SPI HSPI CS=`38` SCK=`41` MISO=`40` MOSI=`39` |

## Critical pins

| Function | GPIO |
|---|---|
| QSPI CS/CLK/D0-D3 | `9, 10, 11, 12, 13, 14` |
| AMOLED RST | `21` |
| I2C SDA/SCL | `47, 48` |
| IMU INT | `46` |
| BOOT button | `0` |

---

## Arduino IDE settings confirmed

| Option | Value |
|---|---|
| Board | `Waveshare ESP32-S3-Touch-AMOLED-1.64` |
| USB CDC On Boot | `Enabled` |
| Partition Scheme | `16M Flash (3MB APP/9.9MB FATFS)` |
| PSRAM | `Enabled` |
| Upload Mode | `UART0 / Hardware CDC` |
| Upload Speed | `921600` |

---

## Current firmware

**Sketch:** `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`

**Architecture proposal:** `ARQUITETURA_TECNICA.md`

**Web variant:** `StratosBrain_WebConfig/StratosBrain_WebConfig.ino`

**Firmware guide:** `FIRMWARE_VARIANTES.md`

## Latest update - 2026-03-27

- The main firmware now includes a single-binary Wi-Fi AP mode instead of requiring a separate test sketch.
- `COMMS` and `CONFIG` can toggle `Wi-Fi AP` and `LoRa` state from the UI.
- When Wi-Fi AP is active, the firmware enters a lighter runtime mode:
  - the old artificial-horizon animation is no longer the center of `PLANE`
  - `PLANE` is being converted into a lighter data panel
  - EFIS refresh now runs only while `PLANE` is the active screen
  - the old artificial-horizon canvas buffer is still released if needed
  - blackbox logging is paused and restored when Wi-Fi is turned off
- AP diagnostics are now visible both on-screen and on the serial monitor:
  - state
  - SSID
  - IP
  - client count
  - web hits
  - heap / PSRAM
- The heaviest blocker was DRAM overflow. This was solved by adding a local LVGL config file:
  - `StratosBrain_LVGL9/lv_conf.h`
  - LVGL now uses CLIB allocation instead of the default 64 KB static pool
  - heavy unused LVGL integrations/widgets were disabled for this product
- Compile validation succeeded locally on 2026-03-27 with:
  - flash: `1242693 bytes (39%)`
  - globals: `48112 bytes (14%)`
- Remaining runtime validation now needs to happen on the real board:
  - AP visibility on phone/PC
  - `PLANE` opening while Wi-Fi stays active
  - `COMMS` portrait fit
  - `CONFIG` first-fold readability
  - AP start/stop stability

## Product direction now locked

The user clarified the product should be organized into 4 real operating modes:

1. `PLANE`
2. `METEO`
3. `COMMS`
4. `CONFIG`

Important interpretation:

- `PLANE` is the aviation mode
- `METEO` is the local weather-station mode
- `METEO` also needs a mobile telemetry variant for balloon use
- `COMMS` is for LoRa/GPS/mesh/debug
- `CONFIG` remains the maintenance and setup area
- the future web UI should mirror these same modes on a phone

This is now the recommended top-level software architecture.

### What is working now

- CO5300 display path is stable with `Arduino_CO5300`
- Direct hardware smoke test remains available in code, but is disabled by default on boot
- FT3168 touch is registered in LVGL as pointer input
- Main menu is live with first-pass top-level screens
- QMI8658 is now polled on a dedicated FreeRTOS task pinned to core `0`
- Touch and IMU share the same I2C bus through a mutex
- EFIS screen now contains:
  - pitch readout
  - roll readout
  - IMU status line
  - first artificial horizon rendered on an LVGL canvas
  - `Setar nivel atual` to zero the horizon using the current position
- Main menu was retuned for the small display:
  - larger touch targets
  - `EFIS` button prioritized
  - easier access with finger input
- A first UX stabilization pass was applied in the main firmware:
  - headers were shortened
  - home was expanded to use more of the portrait area
  - home labels were simplified into a compact status strip
  - the `cockpit wearable` subtitle was removed from the main launcher
  - `PLANE` was compacted for portrait
  - `PLANE` roll sign was inverted in software to match the user's left/right expectation
  - EFIS render was damped visually to feel less sensitive on the screen
  - `CONFIG` was reduced to shorter maintenance blocks
  - `CONFIG` now has a dedicated network block for `WiFi / BLE / WebConfig`
  - `COMMS` now uses large status cards for `WiFi/Web`, `Bluetooth`, `LoRa` and `GPS`
  - secondary screens now share the same `Voltar` style
  - `BOOT` on `GPIO 0` now acts as `Home` in runtime
  - orientation toggle is now intentionally frozen as `portrait fixo` for stability
  - the visual smoke test at boot is now disabled by default
- `CONFIG / TESTE` now contains:
  - display orientation mode toggle
  - sensor refresh button
  - blackbox logger toggle
  - log cleanup button for the SD card
  - known sensor detection summary
  - SD card status and active CSV file
  - raw I2C scan line
- Display orientation now supports:
  - vertical
  - horizontal
  - `AUTO` is intentionally frozen in the current UX pass for stability
- A first SD blackbox logger now runs on a dedicated FreeRTOS task pinned to core `0`
- If a card is present, the firmware now:
  - mounts the SD card on the board HSPI pins
  - creates `/logs/flight_XXXX.csv`
  - logs IMU attitude once per second
  - keeps CSV columns ready for future GPS data
- A lightweight WiFi web mode now exists:
  - it stays in a separate firmware variant to avoid DRAM overflow in the cockpit build
  - the cockpit firmware only shows compact network status
  - if the user wants to see a real `SSID/IP` on other devices, they must flash `StratosBrain_WebConfig.ino`
  - `WebConfig` now prints periodic `SSID / senha / IP / clientes` heartbeats on the serial monitor
  - `WebConfig` now also distinguishes `SoftAP ativo` from `SoftAP nao subiu` directly in the serial boot log
- SD card usage direction is now explicit:
  - SD is for logs, blackbox, telemetry dumps and exported data
  - SD is not a replacement for RAM / PSRAM on ESP32-S3
  - starts a local `SoftAP`
  - shows SSID and IP on-screen
  - exposes a simple browser dashboard on port `80`
  - exposes `/api/status` in JSON
  - but it does **not** fit in the main firmware right now
  - the feature is now guarded by `ENABLE_WIFI_PORTAL 0` in the main sketch
  - this keeps the cockpit build compilable again
- A separate lightweight web firmware now exists:
  - `C:\Esp32\CLAUDE\StratosBrain_WebConfig\StratosBrain_WebConfig.ino`
  - use it to validate `WiFi`, `SoftAP`, browser access, `I2C` and `SD` without loading the cockpit UI
- The current "format SD" action is implemented as a safe log cleanup:
  - it deletes files inside `/logs`
  - it prepares the card for a new mission
  - it does not perform a low-level FAT reformat
  - the confirmation flow was simplified to avoid cramped button text:
    - button label stays fixed
    - the confirmation warning appears in the note/status area
    - confirmation timeout was increased to `8s`

### What is not stable yet

The user reported that the current UI is still too fragile for daily use on the small display.

Current blocking problems:

- text overlap in multiple menus
- `CONFIG` became cramped and hard to read
- `CONFIG` is missing a reliable `Voltar` path back to `HOME`
- switching between vertical and horizontal can corrupt the layout
- some screen changes freeze or partially redraw
- the current layout still feels too much like a prototype screen, not like a wearable/watch UI
- too much information competes in the same small area
- BLE is still not implemented in the main firmware
- the user hit a hard DRAM linker overflow when WiFi was enabled in the main sketch:
  - `.dram0.bss` overflowed by about `30656 bytes`
  - this confirms that web/WiFi must move to a lighter firmware variant

Latest stabilization pass applied in the main sketch:

- `portrait` is now treated as the stable primary layout
- orientation button no longer tries to cycle the whole UI through risky modes
- `BOOT` on `GPIO 0` is now used as runtime `Home/Back`
- `PLANE` portrait layout was compacted to avoid the back button sitting on top of data
- `CONFIG` was rebuilt as a simpler stacked maintenance page instead of dense absolute positioning

Important conclusion:

- do not keep stacking new modules on top of the current navigation model
- the next session should prioritize UI simplification and navigation stability over new sensors
- treat the device as a wearable-style instrument with large actions, shallow navigation and minimal text

### Current IMU approach

- Sensor: `QMI8658`
- Read path: raw accel + gyro over `Wire`
- Filter: simple complementary filter
- Refresh task: `10 ms`
- Current goal: validate motion direction and menu flow on real hardware

This is the first live EFIS step.
It is not yet a full AHRS.
Madgwick fusion with `BMM350` can come later after the basic horizon is validated.

---

## Hardware usage plan

To make better use of the board hardware, the intended mapping is now:

| Hardware | Planned role |
|---|---|
| `QMI8658` | `PLANE` attitude and motion |
| `BMM350` | digital compass / heading |
| `BMP581` | altitude and variometer |
| `AT6558R` | GPS navigation and telemetry |
| `BME688` | weather and air-quality data |
| `LTR-390UV` | UV / ambient light |
| `MAX17048` | battery status |
| microSD | blackbox and mission logging |
| `LoRa CC68` | telemetry uplink, field link and mesh/debug |
| WiFi / BLE | future phone/web setup and mirrored dashboard |

The board should be treated as a compact multi-mode instrument, not as a single-purpose sensor screen.

---

## Latest compile result

Target board:

`esp32:esp32:waveshare_esp32_s3_touch_amoled_164`

Build result on 2026-03-27:

- Flash: `746623 bytes` (`23%`)
- Global variables: `286564 bytes` (`87%`)
- Remaining dynamic memory: `41116 bytes`

Warnings still present:

- Low memory warning from Arduino build
- Linker warning about `_floatdidf.o` missing `.note.GNU-stack`

The sketch compiles successfully despite those warnings.

---

## Root cause already fixed

The original black screen issue was caused by an incorrect `Arduino_CO5300` constructor call.
The panel geometry and offsets were shifted.

The working constructor matches the official `Arduino_GFX` dev-device profile:

```cpp
g_panel = new Arduino_CO5300(
  g_bus,
  21,
  0,
  280,
  456,
  20,
  0,
  180,
  24
);
```

---

## Important tuning flags

### Touch

At the top of the sketch:

- `TOUCH_SWAP_XY`
- `TOUCH_INVERT_X`
- `TOUCH_INVERT_Y`

Use these if touch appears mirrored or rotated.

### IMU

At the top of the sketch:

- `IMU_SWAP_XY`
- `IMU_INVERT_X`
- `IMU_INVERT_Y`
- `IMU_INVERT_Z`

Use these if the artificial horizon moves on the wrong axis or mirrored.
This is expected to need one tuning pass on real hardware.

---

## Suggested runtime architecture

Current direction for the project:

- Core `0`: sensor polling, IMU fusion, logging, radio and background tasks
- Core `1`: LVGL, touch, UI and menu navigation

The current firmware already starts this split by keeping the IMU and SD blackbox logger on dedicated FreeRTOS tasks.

---

## Official vendor material downloaded

Local copy:

`C:\Esp32\CLAUDE\_vendor\ESP32-S3-Touch-AMOLED-1.64-Demo`

Useful references already extracted:

- `Arduino\examples\06_LVGL_Test\FT3168.cpp`
- `Arduino\examples\06_LVGL_Test\FT3168.h`
- `Arduino\examples\02_I2C_QMI8658\qmi8658c.cpp`
- `Arduino\examples\02_I2C_QMI8658\qmi8658c.h`

External visual reference also cloned locally:

`C:\Esp32\CLAUDE\_refs\altitudeIndicator`

This was used only as visual inspiration for the first cockpit-style horizon.

---

## Next recommended steps

1. Freeze new feature work until navigation is stable
2. Redesign the UI around a wearable/watch principle:
   - large buttons
   - little text
   - one clear focal element per screen
   - no dense debug blocks in operational views
3. Rebuild `HOME` as a true `2x2` launcher for:
   - `PLANE`
   - `METEO`
   - `COMMS`
   - `CONFIG`
4. Add a universal `Back` action to every secondary screen
5. Evaluate using the hardware `BOOT` button on GPIO `0` as `Back/Home`
6. Simplify orientation handling before keeping `AUTO`
7. Decide whether the product should:
   - stay portrait-first only for now
   - or support rotation only after the UI is rebuilt
8. Redesign `CONFIG` into a maintenance page with fewer items visible at once
9. Keep the current EFIS as the visual reference, but refactor its layout to be cleaner
10. After the UI base is stable, resume sensor integration in this order:
   - `BMP581` for altitude and vertical speed
   - `BMM350` for compass/heading
   - `AT6558R` for GPS speed and position
   - meteorological sensors in `METEO`
11. Keep the new WiFi AP/dashboard lightweight and avoid adding BLE or a heavy web stack until memory is revisited
12. Prefer a dedicated `WebConfig` sketch instead of forcing WiFi into the cockpit firmware
13. Validate the new `BOOT = Home` runtime shortcut on real hardware
14. Validate portrait and manual landscape only; keep `AUTO` frozen for now
13. Keep `portrait` as the only trusted runtime layout until rotation is redesigned more safely

### Frontend/UX execution order

For the next UI pass, use this concrete order:

1. make the firmware `portrait-first`
2. reduce `HOME` to 4 large buttons with shorter labels
3. replace per-screen ad-hoc back buttons with one shared `Back/Home` pattern
4. shrink or remove subtitles from headers when they steal space
5. split `CONFIG` into only 3 simple groups:
   - tela
   - armazenamento
   - sensores
6. move dense debug text out of the main operational flow
7. only revisit `landscape` and `auto` after `portrait` is stable

Hotspots in the sketch for that work:

- `createHeader()`
- `createNavButton()`
- `createConfigScreen()`
- `createMainScreen()`
- `rebuildUI()`
- `applyDisplayRotation()`
- `handleAutoRotation()`

---

## Immediate stop point for the next session

The project should resume from this decision:

- the hardware foundation is good enough
- the visual foundation is not
- the next session should be a UI reset pass, not a feature sprint

Focus for the next session:

1. clean `HOME`
2. stable `Back/Home` navigation
3. non-breaking orientation strategy
4. readable wearable-sized typography
5. simplified `CONFIG`
6. only then resume `PLANE`, `METEO` and `COMMS`

---

## Morning test checklist

When the user wakes up, the next hardware test should be:

1. flash the current sketch to the board
2. open `PLANE` and confirm the portrait layout no longer overlaps
3. open `CONFIG` and confirm the layout is readable without dense scrolling
4. press the physical `BOOT` button while inside `PLANE`, `COMMS` or `CONFIG`
5. confirm it returns to `HOME`
6. test the orientation button in `CONFIG`
7. confirm it alternates only the manual layout and does not enter `AUTO`
8. insert a microSD card and confirm:
   - SD status changes to mounted
   - logger status looks sane
9. press `Limpar SD` once and confirm:
   - the warning moves to the note area
   - the button text stays stable
10. press `Limpar SD` again within `8s` and confirm:
   - deletion is requested
   - the serial log reflects it
11. only after that, retest navigation:
    - `HOME`
    - `PLANE`
    - `COMMS`
    - `CONFIG`
11. test the runtime `BOOT` button:
    - from any page, it should return to `HOME`
    - on `HOME`, it should stay harmless

---

## Background agent brief

If another agent picks this project up, split the work into these tracks:

### Track A: UI stabilization

Goal:

- stop screen corruption
- stop text overlap
- make navigation consistent

Focus:

- `createHeader()`
- `createNavButton()`
- `createConfigScreen()`
- `createMainScreen()`
- `rebuildUI()`
- `applyDisplayRotation()`
- `handleAutoRotation()`

Expected output:

- identify which layouts exceed the `280x456` budget
- identify where old LVGL screens may still be involved during rebuild/rotation
- propose a portrait-first fallback plan

### Track C: physical BOOT button

Goal:

- evaluate `GPIO 0` as `Back/Home` without hurting boot/programming

Focus:

- runtime read only
- no electrical role changes on the pin
- ignore input during the first `2s` to `3s`
- map only short press

Expected output:

- a safe polling strategy
- a debounce rule
- a single navigation action shared with the on-screen back button

### Track B: runtime sanity

Goal:

- validate the lightweight WiFi approach
- validate SD cleanup flow
- protect the main firmware from more RAM pressure

Focus:

- `startWifiPortal()`
- `handleWifiPortal()`
- `refreshConfigUI()`
- `purgeSdLogsEventCb()`
- blackbox task and SD state transitions

Expected output:

- likely compile/runtime risks from code inspection
- memory-risk notes
- next-step recommendations that do not add heavy frameworks

---

## Notes

- Keep PSRAM enabled
- Do not change the display constructor unless the display path breaks again
- The current EFIS is intentionally lightweight to preserve RAM
- Memory remains tight, so avoid large fonts, full-screen canvases and extra frame buffers unless necessary
- The current blackbox logger stores IMU data at `1 Hz`; GPS columns are already reserved in the CSV
- A first integrated WiFi portal was attempted, but the full UI + WiFi build overflowed DRAM by about `26 KB`
- The current compromise is a lightweight `SoftAP + WiFiServer` dashboard instead of a heavier web framework
- In the current main sketch, WiFi is disabled by default with `#define ENABLE_WIFI_PORTAL 0` because the build overflowed DRAM
- The user explicitly asked to stop compiling locally unless really needed; prefer code edits plus hardware feedback
- The user feedback at the end of this session was:
  - visuals are hard to read
  - texts overlap
  - `CONFIG` lacks a reliable way back
  - screen changes can freeze
  - orientation changes can break the UI
  - the product should feel more like a watch/wearable instrument
- The practical UX decision now is:
  - freeze `AUTO`
  - treat `portrait` as the main layout
  - simplify `CONFIG`
  - unify `Back/Home`
  - evaluate `BOOT` as a runtime shortcut only, not as a boot-time feature
