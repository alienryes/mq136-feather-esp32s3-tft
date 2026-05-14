# MQ-136 H2S Sensor Monitor — ESP32-S3 TFT Feather
**Adafruit Feather ESP32-S3 TFT — Native WiFi + Colour Display**

> Describes [`code.py`](https://github.com/alienryes/mq136-feather-esp32s3-tft/commits/main/code.py) on `main` — click the link to see when the code was last changed.

---

## Overview

Port of the [MQ-136 H2S monitor for Feather RP2350](https://github.com/alienryes/mq136-feather-rp2350) to the Adafruit ESP32-S3 TFT Feather. All application logic is identical; the hardware layer is replaced:

| Feature | RP2350 version | This version |
|---------|---------------|--------------|
| WiFi | AirLift ESP32 SPI co-processor | Built-in ESP32-S3 radio |
| Display | External 128×64 OLED (SH1107) | Built-in 240×135 IPS TFT (ST7789) |
| Calibration | Physical button (FeatherWing A) | HA MQTT command only |
| Time | None | NTP-synced RTC at boot |
| Status LED | None | On-board NeoPixel |

### Inherited from RP2350 version (unchanged)

- **Median-of-3 spike pre-filter** — three rapid ADC reads per sample; the median is passed to the EWMA, making single-sample glitches virtually impossible to propagate
- **EWMA filter** — exponential weighted moving average (α = 2/(N+1), N = 60 default) for smooth readings
- **60-minute history ring buffer** — 12 readings, one per publish cycle; publishes `hour_avg`, `hour_min`, `hour_max`
- **Persistent MQTT connection** — stays connected continuously so commands are received within one sample interval (~10 s)
- **OTA runtime config** — publish interval, EWMA window, and trend threshold all tunable live via MQTT without reflashing
- **Remote commands** — reboot, calibrate, NVM reset, diagnostic publish — all via HA MQTT buttons
- **NVM persistence** — calibration baseline, observed min/max, and config survive reboots
- **Boot publish** — fires immediately after WiFi connects so HA never shows stale data after a reboot
- **Hardware watchdog** — 8-second timeout resets the board if the main loop stalls

---

## Hardware

| Component | Part |
|-----------|------|
| MCU + Display | [Adafruit Feather ESP32-S3 TFT — 4 MB Flash, 2 MB PSRAM](https://www.adafruit.com/product/5483) |
| Sensor | MQ-136 H2S gas sensor module |
| Level shifter | BSS138-based bi-directional level shifter (AOUT 5 V → ADC 3.3 V) |

### Wiring

> **⚠ Level shifter required.**
> The MQ-136 AOUT pin swings to 5 V but the ESP32-S3 ADC input is 3.3 V maximum. A
> bi-directional level shifter (e.g. BSS138-based module) must be placed between
> AOUT and A0 to avoid damaging the ESP32-S3.
> The MQ-136 heater requires 5 V — connect VCC to the Feather **USB/VBUS** pin, not 3.3 V.

```
MQ-136 VCC   → USB/VBUS  (5 V from USB power rail)
MQ-136 GND   → GND
MQ-136 AOUT  → level shifter LV side → A0
               level shifter HV side → 5 V

TFT display  — internal (no external wiring required)
NeoPixel     — internal (board.NEOPIXEL)
```

No FeatherWing, no SPI co-processor, no I2C OLED — just the sensor and level shifter.

---

## Software Requirements

CircuitPython **10.x** for the Feather ESP32-S3 TFT.
Download from <https://circuitpython.org/board/adafruit_feather_esp32s3_tft>

Copy the following libraries from the [Adafruit CircuitPython Library Bundle](https://circuitpython.org/libraries) into `/lib` on the `CIRCUITPY` drive:

| Library | Purpose |
|---------|---------|
| `adafruit_st7789` | ST7789 TFT display driver |
| `adafruit_display_text` | Text labels for displayio |
| `adafruit_bitmap_font` | BDF font loader (for Δ symbol) |
| `adafruit_minimqtt` | MQTT client |
| `adafruit_connection_manager` | Socket/connection management (required by adafruit_minimqtt) |
| `adafruit_ntp` | NTP time sync |
| `adafruit_ticks` | Required by adafruit_minimqtt in CP10 |
| `neopixel` | On-board NeoPixel status LED |

A font file is also required — copy to `/fonts` on `CIRCUITPY`:

| File | Source |
|------|--------|
| `fonts/NotoSans-Regular-12.bdf` | Generate from `NotoSans-Regular.ttf` using `otf2bdf -p 12 -r 75` (see repo `fonts/` folder for pre-generated copy) |

> `NotoSans-Regular-12.bdf` is **not** available as a pre-built download from the Adafruit Bitmap Font repository. The file in this repo was generated with `otf2bdf 3.0` from the `fonts-noto-core` package (Google Noto Fonts, SIL Open Font License).

---

## Installation

1. Flash CircuitPython 10.x onto the Feather ESP32-S3 TFT.
2. Copy the libraries above into `/lib` on `CIRCUITPY`.
3. Copy `settings.toml.example` to `settings.toml` and fill in your values:

   ```toml
   CIRCUITPY_WIFI_SSID     = "your_wifi_ssid"
   CIRCUITPY_WIFI_PASSWORD = "your_wifi_password"

   MQTT_BROKER   = "192.168.x.x"
   MQTT_PORT     = 1883
   MQTT_USERNAME = "your_mqtt_user"
   MQTT_PASSWORD = "your_mqtt_password"
   MQTT_CLIENT   = "mq136"
   ```

4. Copy `code.py` to the root of `CIRCUITPY`.
5. The device starts automatically — no reset required.

---

## Configuration

All tunable parameters are at the top of `code.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SAMPLE_INTERVAL` | `10` s | Seconds between ADC samples (fixed) |
| `_PUBLISH_INTERVAL_DEFAULT` | `300` s | Seconds between MQTT publishes (5 minutes) |
| `_EWMA_N_DEFAULT` | `60` | EWMA equivalent sample count (~10-min smoothing window) |
| `_TREND_THRESHOLD_DEFAULT` | `100` | ADC change between publishes to report Rising or Falling |
| `HOURLY_SIZE` | `12` | Publish cycles in history buffer (12 × 5 min = 60 min) |
| `SPARK_MIN_SPREAD` | `100` | Minimum ADC spread before sparkline auto-scales |
| `NEOPIXEL_BRIGHTNESS` | `0.15` | NeoPixel brightness (0.0–1.0) |
| `CAL_MSG_DURATION` | `3` s | Seconds the "Calibrated" message is shown |
| `WATCHDOG_TIMEOUT` | `8` s | Hardware watchdog timeout |
| `DEVICE_ID` | `"mq136_feather"` | Unique identifier in Home Assistant |

All publish interval, trend threshold, and EWMA window parameters can also be changed live via MQTT without editing code — see [Runtime Configuration](#runtime-configuration).

---

## Display Layout

The 240×135 IPS TFT is divided into seven zones:

```
+--------------------------------------+
|  MQ-136  H2S             12:34:56    |   <- title / NTP clock
+--------------------------------------+
|  14823              Δ:+1823          |   <- raw ADC (large) / delta
|  Avg:13201          RSSI:-65dBm      |   <- hourly avg / RSSI
+--------------------------------------+
|  [  |  |  || |||||||||||||||||||||]  |
|  [  |  |  || |||||||||||||||||||||]  |   <- sparkline (12 cols)
|  . . . . . . . . . . . . . . . . .   |   <- calibrated baseline
|  [  |  |  || |||||||||||||||||||||]  |
+--------------------------------------+
|  Trend: Rising       Nxt Pub:4m32s   |   <- trend / publish countdown
+--------------------------------------+
```

| Zone | Contents |
|------|----------|
| Title row | Device name + NTP clock (UTC) |
| Row 2 | Raw EWMA ADC value (large, colour-coded) + Δ from baseline |
| Row 3 | Hourly average + WiFi RSSI (colour-coded by signal strength) |
| Sparkline | 12 hourly columns, oldest left → newest right; grey line = calibrated baseline |
| Bottom row | Trend (Rising / Falling / Stable) + publish countdown |

### Raw ADC value
Large (scale=2) text, colour-coded by status: green = OK, amber = warming up or publishing, red = MQTT error.

### Delta (Δ:)
Difference between current reading and the calibrated clean-air baseline. Green if below baseline, amber if above by less than `trend_threshold`, red if above by more. Shows `Δ: --` until calibrated.

### Hourly average
Mean of the last 60 minutes of readings. Appears from the second publish cycle onwards. Grey until data is available.

### WiFi RSSI
Current WiFi signal strength, displayed as `RSSI:-67dBm`: green ≥ −67 dBm, amber ≥ −80 dBm, red below −80 dBm. Updated every sample interval from the first sample — visible during warm-up.

### Sparkline
232×40 px graph showing the last 12 publish-cycle readings (oldest left, newest right). Each column is 19px wide with a 1px gap. Appears from the **second publish onwards** (~10 minutes after warm-up at the default 5-minute interval); the area is blank until then. Features:
- **Colour per column** — same green/amber/red scheme as delta, applied individually to each bar
- **White cap** on top of each column to mark the peak clearly
- **Grey horizontal baseline reference line** across the full width when a calibration baseline is set
- **Auto-scaled** — y-axis adjusts to the min/max of visible history; artificially widened if spread is too small (<100 ADC counts) to avoid a flat line at the bottom

### Publish countdown
Shows time until next MQTT publish (e.g. `Nxt Pub:4m32s`). Updates every 10 seconds alongside the sensor sample. Reassures you the device is alive between 5-minute publish gaps.

### NTP clock
Synced once at boot after WiFi connects. Displays UTC time as `HH:MM:SS`. Shows `--:--:--` if NTP sync fails (device still operates normally).

---

## NeoPixel Status LED

The on-board NeoPixel gives at-a-glance status without needing to read the display:

| Colour | Meaning |
|--------|---------|
| Amber | Warming up |
| Green | Last publish successful |
| Red | MQTT failure or error |

Brightness is set by `NEOPIXEL_BRIGHTNESS` (default 0.15 — low enough not to be distracting).

---

## MQTT

### Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `homeassistant/sensor/mq136/state` | Published | JSON state payload (retained) |
| `homeassistant/sensor/mq136/availability` | Published | `online` / `offline` LWT |
| `homeassistant/sensor/mq136/diagnostic` | Published | Extended diagnostic payload (on demand) |
| `homeassistant/sensor/mq136/cmd` | Subscribed | Remote command topic |
| `homeassistant/sensor/mq136/config/#` | Subscribed | Runtime config sub-topics |
| `homeassistant/sensor/mq136_raw/config` | Published | HA discovery (retained) |
| `homeassistant/sensor/mq136_trend/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_delta/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_hour_avg/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_hour_min/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_hour_max/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_status/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_rssi/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_version/config` | Published | HA discovery |
| `homeassistant/sensor/mq136_reset_reason/config` | Published | HA discovery |
| `homeassistant/number/mq136_publish_interval/config` | Published | HA discovery (number entity) |
| `homeassistant/number/mq136_trend_threshold/config` | Published | HA discovery (number entity) |
| `homeassistant/number/mq136_ewma_n/config` | Published | HA discovery (number entity) |
| `homeassistant/button/mq136_reboot/config` | Published | HA discovery (button entity) |
| `homeassistant/button/mq136_calibrate/config` | Published | HA discovery (button entity) |
| `homeassistant/button/mq136_nvm_reset/config` | Published | HA discovery (button entity) |
| `homeassistant/button/mq136_status/config` | Published | HA discovery (button entity) |

### State payload

Published every `publish_interval` seconds after warmup:

```json
{
  "raw": 14823,
  "trend": "Stable",
  "status": "OK",
  "version": "1.0.0",
  "reset_reason": "Power On",
  "rssi": -65,
  "delta": 1823,
  "hour_avg": 13201,
  "hour_min": 12950,
  "hour_max": 14823
}
```

| Field | Type | Description |
|-------|------|-------------|
| `raw` | int | EWMA ADC count (0–65535) |
| `trend` | string | `Rising`, `Falling`, or `Stable` |
| `status` | string | `OK` (normal) or `Warming Up` (boot publish only) |
| `version` | string | Firmware version string |
| `reset_reason` | string | Why the board last reset (e.g. `Power On`, `Watchdog`, `Software`) |
| `rssi` | int | WiFi signal strength in dBm |
| `delta` | int | Raw minus clean-air baseline — only present after calibration |
| `hour_avg` | int | Average EWMA over history buffer — omitted until first publish |
| `hour_min` | int | Minimum EWMA in history buffer — omitted until first publish |
| `hour_max` | int | Maximum EWMA in history buffer — omitted until first publish |

### Boot publish

Fires immediately after WiFi connects and NTP syncs, before warmup completes:

```json
{"raw": 13100, "status": "Warming Up", "version": "1.0.0", "reset_reason": "Power On"}
```

---

## Home Assistant

With MQTT auto-discovery enabled (the default in modern HA), the device appears as **MQ-136 H2S Sensor** with these entities:

### Sensor entities

| Entity | Type | Notes |
|--------|------|-------|
| MQ-136 Raw ADC | Measurement | Integer EWMA ADC count |
| MQ-136 Trend | Text | Rising / Falling / Stable |
| MQ-136 Delta | Measurement | Raw minus baseline; shows 0 until calibrated |
| MQ-136 Hourly Average | Measurement | Mean over last 60 minutes |
| MQ-136 Hourly Min | Measurement | Minimum over last 60 minutes |
| MQ-136 Hourly Max | Measurement | Maximum over last 60 minutes |
| MQ-136 Status | Diagnostic | `Warming Up` on boot, `OK` thereafter |
| MQ-136 WiFi RSSI | Diagnostic / Measurement | dBm |
| MQ-136 Firmware Version | Diagnostic | Version string |
| MQ-136 Reset Reason | Diagnostic | e.g. `Power On`, `Watchdog`, `Software` |

### Config entities (number)

| Entity | Range | Step | Default |
|--------|-------|------|---------|
| MQ-136 Publish Interval | 60–1800 s | 10 | 300 s |
| MQ-136 Trend Threshold | 10–5000 | 10 | 100 |
| MQ-136 EWMA Window | 2–120 | 1 | 60 |

### Command entities (button)

| Entity | Action |
|--------|--------|
| MQ-136 Reboot | Clean software reboot |
| MQ-136 Calibrate | Capture current reading as clean-air baseline |
| MQ-136 NVM Reset | Clear all persisted min/max, baseline, and config |
| MQ-136 Diagnostic Publish | Immediate publish of extended diagnostic payload |

---

## Runtime Configuration

Config changes take effect immediately without reflashing and are persisted to NVM so they survive reboots.

### Via Home Assistant

Use the **MQ-136 Publish Interval**, **MQ-136 Trend Threshold**, and **MQ-136 EWMA Window** number entities in the HA device page.

### Via MQTT directly

```
homeassistant/sensor/mq136/config/publish_interval  →  300
homeassistant/sensor/mq136/config/trend_threshold   →  150
homeassistant/sensor/mq136/config/ewma_n            →  90
```

Values are clamped to their allowed ranges automatically.

---

## Baseline Calibration

Calibration sets the clean-air baseline used for the `delta` field and the sparkline colour coding. There is no physical button — calibration is done via Home Assistant or MQTT.

1. Let the sensor warm up fully (Status: OK on display, green NeoPixel).
2. Ensure the sensor is in clean, uncontaminated air.
3. Press **MQ-136 Calibrate** in the HA device page (or publish `calibrate` to `homeassistant/sensor/mq136/cmd`).
4. The display shows `Calibrated` for 3 seconds to confirm.

The baseline is written to NVM immediately and survives power cycles.

Once calibrated:
- `delta` (raw minus baseline) is included in every MQTT publish
- The sparkline colours each column relative to the baseline (green = below, amber = above, red = well above)
- A grey horizontal reference line is drawn across the sparkline at the baseline level

To clear the baseline via MQTT: press **MQ-136 NVM Reset** in HA (this also clears observed min/max and config — the board reboots with factory defaults).

---

## NVM Persistence

NVM layout is byte-identical to the RP2350 version — boards can be swapped without losing calibration data.

| Bytes | Contents |
|-------|----------|
| 0–1 | Sensor magic `0xA5 0x5A` |
| 2–3 | Observed min (big-endian uint16) |
| 4–5 | Observed max (big-endian uint16) |
| 6 | Baseline valid flag (`0x01` = valid) |
| 7–8 | Baseline value (big-endian uint16) |
| 9–10 | Config magic `0xC0 0xDE` |
| 11–12 | Publish interval (big-endian uint16, seconds) |
| 13–14 | Trend threshold (big-endian uint16) |
| 15–16 | EWMA N (big-endian uint16) |
| 17–19 | Reserved (zeroed) |

To reset all persisted values from the CircuitPython REPL:

```python
import microcontroller
microcontroller.nvm[0:2] = b'\x00\x00'   # wipe sensor region
microcontroller.nvm[9:11] = b'\x00\x00'  # wipe config region
```

---

## Serial Output

Connect via USB serial (115200 baud).

**Boot:**
```
NVM sensor  lo=12950  hi=14823  baseline=13000
NVM config  interval=300  threshold=100  ewma_n=60
Connecting to WiFi...
Connected  IP: 192.168.1.x
NTP synced
MQTT connecting...
MQTT connected
Boot publish  raw=13100  reset_reason=Power On
```

**Warmup** (every 10 s):
```
Warming up  n=1/6  raw=13100
Warming up  n=2/6  raw=13201
```

**Normal operation** (every 10 s):
```
raw=14823  lo=12950  hi=14823  status=OK  trend=Stable  delta=1823
```

**Each publish** (every 5 min):
```
Hourly  avg=13201  min=12950  max=14823
Published  raw=14823  trend=Stable  rssi=-65 dBm
```

**Remote commands:**
```
CMD received: calibrate
Baseline calibrated: 13000
CMD received: reboot
Remote reboot requested
```

---

## Signal Processing

### Median-of-3 pre-filter

Each sample takes three rapid ADC readings and returns the middle value. This eliminates single-sample spikes (including the occasional ADC zero-drop seen in raw data) before they reach the EWMA. A spike must appear in at least two of the three reads to influence the output.

### EWMA filter

The median value is fed into an exponential weighted moving average:

```
EWMA = α × sample + (1 − α) × EWMA_prev
α = 2 / (N + 1)
```

At the default N = 60, α ≈ 0.033 — each new sample contributes ~3.3% of the output, giving a ~10-minute effective smoothing window at 10 s/sample. N is runtime-tunable via MQTT (range 2–120).

---

## Files

| File | Description |
|------|-------------|
| `code.py` | Main application |
| `settings.toml` | WiFi and MQTT credentials (gitignored) |
| `settings.toml.example` | Template — copy to `settings.toml` and fill in |
| `README.md` | This file |
