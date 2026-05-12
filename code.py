#
# ESP32-S3 TFT Feather version — ported from mq136-feather-rp2350.
#
# Key differences from RP2350 version:
#
#   Native WiFi  — uses the ESP32-S3's built-in radio directly via the
#                   CircuitPython wifi/socketpool/ssl modules. No AirLift
#                   co-processor or adafruit_esp32spi library required.
#
#   Colour TFT   — built-in 240x135 IPS ST7789 display replaces the external
#                   SH1107 OLED. Status text is colour-coded (green/amber/red).
#                   Bar chart scaled to the wider display.
#
#   No button    — physical calibration button removed; calibration is handled
#                   exclusively via the HA MQTT command (cmd/calibrate).
#
# Inherited from RP2350 version (unchanged):
#   EWMA filter with median-of-3 spike pre-filter
#   Hourly min/avg/max ring buffer
#   Full HA MQTT auto-discovery
#   Runtime-tunable config via MQTT (publish interval, EWMA window, threshold)
#   Remote commands: reboot, calibrate, NVM reset, identify, diagnostic publish
#   NVM persistence of calibration, min/max, and config across reboots
#   Dual-core: Core 0 owns sampling/display, Core 1 owns WiFi/MQTT
#
# Source repo:  https://github.com/alienryes/mq136-feather-esp32s3-tft
# RP2350 repo:  https://github.com/alienryes/mq136-feather-rp2350
#
# Hardware connections:
#   MQ-136 AOUT  → A0
#   TFT display  — internal (no wiring required)
#
# Required CircuitPython libraries (copy to /lib on CIRCUITPY):
#   adafruit_st7789
#   adafruit_display_text
#   adafruit_bitmap_font
#   adafruit_minimqtt
#   adafruit_ntp
#   adafruit_ticks          (required by adafruit_minimqtt in CP10)
#   neopixel
#
# Required font file (copy to /fonts on CIRCUITPY):
#   fonts/NotoSans-Regular-12.bdf   (for Δ delta symbol on display)
#   Source: https://github.com/adafruit/Adafruit_CircuitPython_Bitmap_Font/tree/main/examples/fonts

import supervisor
supervisor.runtime.autoreload = False

try:
    import _thread
    _DUAL_CORE = True
except ImportError:
    _DUAL_CORE = False

import gc
import json
import os
import ssl
import struct
import time
import board
import analogio
import displayio
import fourwire
import terminalio
import microcontroller
import watchdog
import wifi
import socketpool
import rtc
from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font
import adafruit_ntp
import adafruit_st7789
import neopixel
import adafruit_minimqtt.adafruit_minimqtt as MQTT

# ---------------------------------------------------------------------------
# Firmware version
# ---------------------------------------------------------------------------

FIRMWARE_VERSION = "1.0.0"

# ---------------------------------------------------------------------------
# Reset reason — captured once at boot before anything can change it
# ---------------------------------------------------------------------------

try:
    _RESET_REASON = str(microcontroller.cpu.reset_reason).split(".")[-1].replace("_", " ").title()
except Exception:
    _RESET_REASON = "Unknown"

# ---------------------------------------------------------------------------
# Configuration — compile-time defaults
# ---------------------------------------------------------------------------

SENSOR_PIN = board.A0
DEVICE_ID = "mq136_feather"
BASE_TOPIC = "homeassistant/sensor/mq136"
STATE_TOPIC = BASE_TOPIC + "/state"
AVAIL_TOPIC = BASE_TOPIC + "/availability"
DIAG_TOPIC = BASE_TOPIC + "/diagnostic"
CMD_TOPIC = BASE_TOPIC + "/cmd"
CFG_TOPIC = BASE_TOPIC + "/config"

SAMPLE_INTERVAL = 10     # seconds between individual ADC samples (fixed)

# Runtime-tunable parameters — defaults used until NVM provides overrides.
_PUBLISH_INTERVAL_DEFAULT = 300
_TREND_THRESHOLD_DEFAULT = 100
_EWMA_N_DEFAULT = 60

# Publish interval bounds (seconds): 60 s minimum, 30 min maximum.
PUBLISH_INTERVAL_MIN = 60
PUBLISH_INTERVAL_MAX = 1800

# Trend threshold bounds (ADC counts).
TREND_THRESHOLD_MIN = 10
TREND_THRESHOLD_MAX = 5000

# EWMA N bounds (equivalent sample count).
EWMA_N_MIN = 2
EWMA_N_MAX = 120

# 12-entry history: one slot per publish cycle = 60 minutes at 5-minute interval.
HOURLY_SIZE = 12

WATCHDOG_TIMEOUT = 8     # seconds — hardware maximum
ADC_MAX = 65535
CAL_MSG_DURATION = 3     # seconds to show "Calibrated" message
IDENTIFY_FLASHES = 6     # number of display on/off cycles for identify

# ---------------------------------------------------------------------------
# Display geometry (240x135 ST7789)
# ---------------------------------------------------------------------------

TFT_WIDTH = 240
TFT_HEIGHT = 135
SPARK_X = 4             # sparkline left margin
SPARK_Y = 68            # sparkline top (just below divider line 2)
SPARK_W = 232           # sparkline pixel width
SPARK_H = 44            # sparkline pixel height (down to y=112, leaving label row)
SPARK_COLS = 12         # one column per hourly reading slot
SPARK_COL_W = SPARK_W // SPARK_COLS   # 19px per column
SPARK_MIN_SPREAD = 100  # minimum value spread before scaling activates

# Colours
COL_BG = 0x000000
COL_WHITE = 0xFFFFFF
COL_GREEN = 0x00CC44
COL_AMBER = 0xFFAA00
COL_RED = 0xFF2222
COL_BLUE = 0x4488FF
COL_GREY = 0x888888

# NeoPixel brightness (0.0–1.0)
NEOPIXEL_BRIGHTNESS = 0.15

# ---------------------------------------------------------------------------
# NVM layout (identical to RP2350 version — data is cross-compatible)
# ---------------------------------------------------------------------------
#
# Sensor data region [0:9]:
#   [0:2]  magic           0xA5 0x5A
#   [2:4]  obs_min         big-endian uint16
#   [4:6]  obs_max         big-endian uint16
#   [6]    baseline flag   0x01 = valid
#   [7:9]  baseline        big-endian uint16
#
# Config region [9:20]:
#   [9:11]  config magic   0xC0 0xDE
#   [11:13] publish_interval  big-endian uint16 (seconds)
#   [13:15] trend_threshold   big-endian uint16
#   [15:17] ewma_n            big-endian uint16
#   [17:20] reserved (zeroed)

_NVM_MAGIC = b'\xa5\x5a'
_NVM_CFG_MAGIC = b'\xc0\xde'
_NVM_SENSOR_SIZE = 9
_NVM_CFG_OFFSET = 9
_NVM_CFG_SIZE = 11   # magic(2) + publish_interval(2) + trend_threshold(2) + ewma_n(2) + reserved(3)

# ---------------------------------------------------------------------------
# Runtime config state (populated from NVM then overridable via MQTT)
# ---------------------------------------------------------------------------

publish_interval = _PUBLISH_INTERVAL_DEFAULT
trend_threshold = _TREND_THRESHOLD_DEFAULT
ewma_n = _EWMA_N_DEFAULT
ewma_alpha = 2.0 / (ewma_n + 1)
warmup_samples = 6   # always 60 s at 10 s/sample; not user-tunable

# ---------------------------------------------------------------------------
# Home Assistant MQTT discovery
# ---------------------------------------------------------------------------

_DEVICE_INFO = {
    "identifiers": [DEVICE_ID],
    "name": "MQ-136 H2S Sensor",
    "model": "Feather ESP32-S3 TFT + MQ-136",
    "manufacturer": "Adafruit",
}


def _sensor_discovery(name, uid, value_tpl, icon, extra=None):
    doc = {
        "name": name,
        "unique_id": DEVICE_ID + "_" + uid,
        "state_topic": STATE_TOPIC,
        "availability_topic": AVAIL_TOPIC,
        "value_template": value_tpl,
        "icon": icon,
        "device": _DEVICE_INFO,
    }
    if extra:
        doc.update(extra)
    return json.dumps(doc)


def _number_discovery(name, uid, cmd_topic, min_val, max_val, step, icon, unit=""):
    doc = {
        "name": name,
        "unique_id": DEVICE_ID + "_" + uid,
        "command_topic": cmd_topic,
        "min": min_val,
        "max": max_val,
        "step": step,
        "icon": icon,
        "entity_category": "config",
        "device": _DEVICE_INFO,
    }
    if unit:
        doc["unit_of_measurement"] = unit
    return json.dumps(doc)


def _button_discovery(name, uid, cmd_topic, payload, icon):
    doc = {
        "name": name,
        "unique_id": DEVICE_ID + "_" + uid,
        "command_topic": cmd_topic,
        "payload_press": payload,
        "icon": icon,
        "entity_category": "config",
        "device": _DEVICE_INFO,
    }
    return json.dumps(doc)


def build_discovery_topics():
    topics = {}

    topics["homeassistant/sensor/mq136_raw/config"] = _sensor_discovery(
        "MQ-136 Raw ADC", "raw", "{{ value_json.raw }}", "mdi:gauge",
        {"state_class": "measurement"},
    )
    topics["homeassistant/sensor/mq136_trend/config"] = _sensor_discovery(
        "MQ-136 Trend", "trend", "{{ value_json.trend }}", "mdi:trending-up",
    )
    topics["homeassistant/sensor/mq136_delta/config"] = _sensor_discovery(
        "MQ-136 Delta", "delta", "{{ value_json.delta | int(0) }}", "mdi:delta",
        {"state_class": "measurement"},
    )
    topics["homeassistant/sensor/mq136_hour_avg/config"] = _sensor_discovery(
        "MQ-136 Hourly Average", "hour_avg",
        "{{ value_json.hour_avg | int(0) }}", "mdi:chart-timeline-variant",
        {"state_class": "measurement"},
    )
    topics["homeassistant/sensor/mq136_hour_min/config"] = _sensor_discovery(
        "MQ-136 Hourly Min", "hour_min",
        "{{ value_json.hour_min | int(0) }}", "mdi:chart-timeline-variant",
        {"state_class": "measurement"},
    )
    topics["homeassistant/sensor/mq136_hour_max/config"] = _sensor_discovery(
        "MQ-136 Hourly Max", "hour_max",
        "{{ value_json.hour_max | int(0) }}", "mdi:chart-timeline-variant",
        {"state_class": "measurement"},
    )
    topics["homeassistant/sensor/mq136_status/config"] = _sensor_discovery(
        "MQ-136 Status", "status",
        "{{ value_json.status | default('OK') }}", "mdi:information-outline",
        {"entity_category": "diagnostic"},
    )
    topics["homeassistant/sensor/mq136_rssi/config"] = _sensor_discovery(
        "MQ-136 WiFi RSSI", "rssi", "{{ value_json.rssi }}", "mdi:wifi",
        {
            "unit_of_measurement": "dBm",
            "device_class": "signal_strength",
            "state_class": "measurement",
            "entity_category": "diagnostic",
        },
    )
    topics["homeassistant/sensor/mq136_version/config"] = _sensor_discovery(
        "MQ-136 Firmware Version", "version",
        "{{ value_json.version }}", "mdi:tag",
        {"entity_category": "diagnostic"},
    )
    topics["homeassistant/sensor/mq136_reset_reason/config"] = _sensor_discovery(
        "MQ-136 Reset Reason", "reset_reason",
        "{{ value_json.reset_reason }}", "mdi:restart",
        {"entity_category": "diagnostic"},
    )
    topics["homeassistant/number/mq136_publish_interval/config"] = _number_discovery(
        "MQ-136 Publish Interval", "publish_interval_cfg",
        CFG_TOPIC + "/publish_interval",
        PUBLISH_INTERVAL_MIN, PUBLISH_INTERVAL_MAX, 10,
        "mdi:timer-outline", "s",
    )
    topics["homeassistant/number/mq136_trend_threshold/config"] = _number_discovery(
        "MQ-136 Trend Threshold", "trend_threshold_cfg",
        CFG_TOPIC + "/trend_threshold",
        TREND_THRESHOLD_MIN, TREND_THRESHOLD_MAX, 10,
        "mdi:swap-vertical",
    )
    topics["homeassistant/number/mq136_ewma_n/config"] = _number_discovery(
        "MQ-136 EWMA Window", "ewma_n_cfg",
        CFG_TOPIC + "/ewma_n",
        EWMA_N_MIN, EWMA_N_MAX, 1,
        "mdi:chart-bell-curve",
    )
    topics["homeassistant/button/mq136_reboot/config"] = _button_discovery(
        "MQ-136 Reboot", "cmd_reboot",
        CMD_TOPIC, "reboot", "mdi:restart",
    )
    topics["homeassistant/button/mq136_calibrate/config"] = _button_discovery(
        "MQ-136 Calibrate", "cmd_calibrate",
        CMD_TOPIC, "calibrate", "mdi:target",
    )
    topics["homeassistant/button/mq136_nvm_reset/config"] = _button_discovery(
        "MQ-136 NVM Reset", "cmd_nvm_reset",
        CMD_TOPIC, "nvm_reset", "mdi:database-remove",
    )
    topics["homeassistant/button/mq136_identify/config"] = _button_discovery(
        "MQ-136 Identify", "cmd_identify",
        CMD_TOPIC, "identify", "mdi:led-on",
    )
    topics["homeassistant/button/mq136_status/config"] = _button_discovery(
        "MQ-136 Diagnostic Publish", "cmd_status",
        CMD_TOPIC, "status", "mdi:clipboard-pulse",
    )
    return topics


# ---------------------------------------------------------------------------
# Watchdog
# ---------------------------------------------------------------------------

microcontroller.watchdog.timeout = WATCHDOG_TIMEOUT
microcontroller.watchdog.mode = watchdog.WatchDogMode.RESET


def pat_watchdog():
    microcontroller.watchdog.feed()


# ---------------------------------------------------------------------------
# Sensor — EWMA filter with median-of-3 spike pre-filter
# ---------------------------------------------------------------------------

sensor_pin = analogio.AnalogIn(SENSOR_PIN)

_ewma = 0.0
_sample_count = 0
_obs_min = ADC_MAX
_obs_max = 0
_nvm_dirty = False


def _median3():
    """Return the median of three rapid ADC readings (spike pre-filter)."""
    a = sensor_pin.value
    b = sensor_pin.value
    c = sensor_pin.value
    if a <= b <= c or c <= b <= a:
        return b
    if b <= a <= c or c <= a <= b:
        return a
    return c


def sample_sensor():
    global _ewma, _sample_count, _obs_min, _obs_max, _nvm_dirty
    value = _median3()
    if _sample_count == 0:
        _ewma = float(value)
    else:
        _ewma = ewma_alpha * value + (1.0 - ewma_alpha) * _ewma
    _sample_count += 1
    if value < _obs_min:
        _obs_min = value
        _nvm_dirty = True
    if value > _obs_max:
        _obs_max = value
        _nvm_dirty = True


def current_reading():
    return int(_ewma)


def warmed_up():
    return _sample_count >= warmup_samples


def trend_symbol(prev_raw, curr_raw):
    delta = curr_raw - prev_raw
    if delta > trend_threshold:
        return "Rising"
    if delta < -trend_threshold:
        return "Falling"
    return "Stable"


# ---------------------------------------------------------------------------
# 60-minute history ring buffer
# ---------------------------------------------------------------------------

_hourly = [0] * HOURLY_SIZE
_hourly_pos = 0
_hourly_count = 0


def record_hourly(value):
    global _hourly_pos, _hourly_count
    _hourly[_hourly_pos] = value
    _hourly_pos = (_hourly_pos + 1) % HOURLY_SIZE
    if _hourly_count < HOURLY_SIZE:
        _hourly_count += 1


def hourly_stats():
    """Return (avg, min, max) over stored history, or None if no data yet."""
    if _hourly_count == 0:
        return None
    if _hourly_count < HOURLY_SIZE:
        active = _hourly[:_hourly_count]
    else:
        active = _hourly[_hourly_pos:] + _hourly[:_hourly_pos]
    return sum(active) // _hourly_count, min(active), max(active)


# ---------------------------------------------------------------------------
# NVM persistence
# ---------------------------------------------------------------------------

_baseline = 0
_baseline_valid = False


def _nvm_load():
    global _obs_min, _obs_max, _baseline, _baseline_valid
    global publish_interval, trend_threshold, ewma_n, ewma_alpha
    data = bytes(microcontroller.nvm[0:_NVM_SENSOR_SIZE])
    if data[0:2] == _NVM_MAGIC:
        _obs_min, _obs_max, bv, _baseline = struct.unpack(">HHBH", data[2:])
        _baseline_valid = (bv == 0x01)
        baseline_str = ("  baseline=" + str(_baseline)) if _baseline_valid else "  no baseline"
        print("NVM sensor  lo=" + str(_obs_min) + "  hi=" + str(_obs_max) + baseline_str)
    else:
        print("NVM sensor: no valid data")
    cfg = bytes(microcontroller.nvm[_NVM_CFG_OFFSET:_NVM_CFG_OFFSET + _NVM_CFG_SIZE])
    if cfg[0:2] == _NVM_CFG_MAGIC:
        pi, tt, en = struct.unpack(">HHH", cfg[2:8])
        publish_interval = max(PUBLISH_INTERVAL_MIN, min(pi, PUBLISH_INTERVAL_MAX))
        trend_threshold = max(TREND_THRESHOLD_MIN, min(tt, TREND_THRESHOLD_MAX))
        ewma_n = max(EWMA_N_MIN, min(en, EWMA_N_MAX))
        ewma_alpha = 2.0 / (ewma_n + 1)
        print("NVM config  interval=" + str(publish_interval)
              + "  threshold=" + str(trend_threshold)
              + "  ewma_n=" + str(ewma_n))
    else:
        print("NVM config: no valid data, using defaults")


def _nvm_write_sensor():
    global _nvm_dirty
    bv = 0x01 if _baseline_valid else 0x00
    data = _NVM_MAGIC + struct.pack(">HHBH", _obs_min, _obs_max, bv, _baseline)
    microcontroller.nvm[0:_NVM_SENSOR_SIZE] = data
    _nvm_dirty = False


def _nvm_write_config():
    cfg = _NVM_CFG_MAGIC + struct.pack(">HHH", publish_interval, trend_threshold, ewma_n)
    cfg += b'\x00' * 3
    microcontroller.nvm[_NVM_CFG_OFFSET:_NVM_CFG_OFFSET + _NVM_CFG_SIZE] = cfg
    print("NVM config saved  interval=" + str(publish_interval)
          + "  threshold=" + str(trend_threshold)
          + "  ewma_n=" + str(ewma_n))


def nvm_reset_all():
    microcontroller.nvm[0:2] = b'\x00\x00'
    microcontroller.nvm[_NVM_CFG_OFFSET:_NVM_CFG_OFFSET + 2] = b'\x00\x00'
    print("NVM reset")


_nvm_load()


# ---------------------------------------------------------------------------
# Baseline calibration
# ---------------------------------------------------------------------------


def calibrate_baseline():
    global _baseline, _baseline_valid
    if not warmed_up():
        return False
    _baseline = current_reading()
    _baseline_valid = True
    _nvm_write_sensor()
    print("Baseline calibrated: " + str(_baseline))
    return True


# ---------------------------------------------------------------------------
# ST7789 TFT 240x135 — colour display
# ---------------------------------------------------------------------------

displayio.release_displays()
tft_cs = board.TFT_CS
tft_dc = board.TFT_DC
spi = board.SPI()
display_bus = fourwire.FourWire(spi, command=tft_dc, chip_select=tft_cs)
display = adafruit_st7789.ST7789(
    display_bus,
    width=TFT_WIDTH,
    height=TFT_HEIGHT,
    rowstart=40,
    colstart=53,
    rotation=270,
)
display.auto_refresh = False

# Load a font that includes the delta symbol (Δ, U+0394).
# Falls back to terminalio.FONT if the file is missing.
try:
    _FONT_DELTA = bitmap_font.load_font("/fonts/NotoSans-Regular-12.bdf")
except Exception:
    _FONT_DELTA = terminalio.FONT

# Palette: 0=black, 1=white, 2=green, 3=amber, 4=red, 5=blue, 6=grey
_pal = displayio.Palette(7)
_pal[0] = COL_BG
_pal[1] = COL_WHITE
_pal[2] = COL_GREEN
_pal[3] = COL_AMBER
_pal[4] = COL_RED
_pal[5] = COL_BLUE
_pal[6] = COL_GREY


def _status_colour(status):
    """Return palette index for a given status string."""
    s = status.lower()
    if "warm" in s or "publish" in s:
        return 3   # amber
    if "no mqtt" in s or "fail" in s or "error" in s:
        return 4   # red
    return 2       # green


def _hline_bitmap(width, colour_idx=6):
    bm = displayio.Bitmap(width, 1, 7)
    for x in range(width):
        bm[x, 0] = colour_idx
    return bm


# ---------------------------------------------------------------------------
# Display layout (240x135):
#
#  y=  8  MQ-136  H2S              12:34:56
#  y= 20  ──────────────────────────────────
#  y= 38  14823 (×2)                D:+1823
#  y= 55  Avg:13201                 -67dBm
#  y= 66  ──────────────────────────────────
#  y= 68  sparkline (232×44px, 12 columns)
#  y=117  Rising                  Nxt:4m32s
# ---------------------------------------------------------------------------

splash = displayio.Group()

# Title (left) + clock (right)
splash.append(label.Label(
    terminalio.FONT, text="MQ-136  H2S",
    color=COL_WHITE, x=4, y=8, scale=2,
))
_lbl_clock = label.Label(
    terminalio.FONT, text="--:--:--", color=COL_GREY, x=168, y=8, scale=1,
)
splash.append(_lbl_clock)

# Divider line 1
splash.append(displayio.TileGrid(
    _hline_bitmap(TFT_WIDTH), pixel_shader=_pal, x=0, y=20,
))

# Raw ADC value (left, large) + delta (right)
_lbl_raw = label.Label(
    terminalio.FONT, text="------", color=COL_BLUE, x=4, y=38, scale=2,
)
splash.append(_lbl_raw)
_lbl_delta = label.Label(
    _FONT_DELTA, text="\u0394: ------", color=COL_GREY, x=148, y=38,
)
splash.append(_lbl_delta)

# Hourly average (left) + RSSI (right)
_lbl_avg = label.Label(
    terminalio.FONT, text="Avg: -----", color=COL_GREY, x=4, y=55,
)
splash.append(_lbl_avg)
_lbl_rssi = label.Label(
    terminalio.FONT, text="----dBm", color=COL_GREY, x=168, y=55,
)
splash.append(_lbl_rssi)

# Divider line 2
splash.append(displayio.TileGrid(
    _hline_bitmap(TFT_WIDTH), pixel_shader=_pal, x=0, y=66,
))

# Sparkline bitmap (replaces bar chart)
_spark_bm = displayio.Bitmap(SPARK_W, SPARK_H, 7)
_spark_bm.fill(0)
splash.append(displayio.TileGrid(_spark_bm, pixel_shader=_pal, x=SPARK_X, y=SPARK_Y))

# Trend (left) + publish countdown (right)
_lbl_trend = label.Label(terminalio.FONT, text="Trend: ---", color=COL_GREY, x=4, y=117)
splash.append(_lbl_trend)
_lbl_next = label.Label(terminalio.FONT, text="Next: --:--", color=COL_GREY, x=140, y=117)
splash.append(_lbl_next)

display.root_group = splash


def _fmt_countdown(secs):
    """Format seconds as Mm Ss or Ss."""
    s = max(0, int(secs))
    if s >= 60:
        return str(s // 60) + "m" + str(s % 60).zfill(2) + "s"
    return str(s) + "s"


def draw_display(raw, status, trend="---", show_prefix=True,
                 hour_avg=None, rssi=0, next_secs=None, clock_str=None):
    col = _status_colour(status)
    _lbl_raw.text = str(raw)
    _lbl_raw.color = _pal[col]

    # Delta from baseline
    if _baseline_valid:
        d = raw - _baseline
        _lbl_delta.text = "\u0394:" + ("+" if d >= 0 else "") + str(d)
        _lbl_delta.color = _pal[4 if d > trend_threshold else (3 if d > 0 else 2)]
    else:
        _lbl_delta.text = "\u0394: --"
        _lbl_delta.color = _pal[6]

    # Hourly average
    if hour_avg is not None:
        _lbl_avg.text = "Avg:" + str(hour_avg)
        _lbl_avg.color = _pal[1]
    else:
        _lbl_avg.text = "Avg: --"
        _lbl_avg.color = _pal[6]

    # RSSI
    _lbl_rssi.text = str(rssi) + "dBm"
    _lbl_rssi.color = _pal[2 if rssi >= -67 else (3 if rssi >= -80 else 4)]

    # Trend
    _lbl_trend.text = ("" if show_prefix else "") + trend
    _lbl_trend.color = _pal[3 if trend == "Rising" else (2 if trend == "Falling" else 1)]

    # Publish countdown
    if next_secs is not None:
        _lbl_next.text = "Nxt:" + _fmt_countdown(next_secs)
        _lbl_next.color = _pal[1]
    else:
        _lbl_next.text = status if not show_prefix else "Status:" + status
        _lbl_next.color = _pal[col]

    # Clock
    if clock_str:
        _lbl_clock.text = clock_str
        _lbl_clock.color = _pal[1]

    # Sparkline
    _spark_bm.fill(0)
    active = (_hourly[:_hourly_count] if _hourly_count < HOURLY_SIZE
              else _hourly[_hourly_pos:] + _hourly[:_hourly_pos])
    if len(active) >= 2:
        lo = min(active)
        hi = max(active)
        spread = hi - lo
        if spread < SPARK_MIN_SPREAD:
            lo = lo - SPARK_MIN_SPREAD // 2
            spread = SPARK_MIN_SPREAD
        # Draw baseline reference line (grey) if calibrated
        if _baseline_valid:
            b_frac = (_baseline - lo) / spread
            b_y = int((1.0 - b_frac) * (SPARK_H - 1))
            b_y = max(0, min(b_y, SPARK_H - 1))
            for x in range(SPARK_W):
                _spark_bm[x, b_y] = 6   # grey
        # Draw column for each reading, newest rightmost
        for i, val in enumerate(active):
            frac = (val - lo) / spread
            bar_h = max(1, int(frac * (SPARK_H - 1)))
            # Colour column by delta from baseline
            if _baseline_valid:
                d = val - _baseline
                c = 4 if d > trend_threshold else (3 if d > 0 else 2)
            else:
                c = col
            x0 = i * SPARK_COL_W
            x1 = x0 + SPARK_COL_W - 1   # 1px gap between columns
            for x in range(x0, x1):
                for y in range(SPARK_H - bar_h, SPARK_H):
                    _spark_bm[x, y] = c
            # Bright top pixel to mark the peak of each column
            top_y = SPARK_H - bar_h
            for x in range(x0, x1):
                _spark_bm[x, top_y] = 1   # white cap

    pat_watchdog()
    display.refresh()
    pat_watchdog()


def identify_flash():
    """Flash the display on/off for physical identification."""
    for _ in range(IDENTIFY_FLASHES):
        display.brightness = 0
        time.sleep(0.2)
        pat_watchdog()
        display.brightness = 1.0
        time.sleep(0.2)
        pat_watchdog()
    print("Identify flash complete")


# ---------------------------------------------------------------------------
# NeoPixel status indicator
# ---------------------------------------------------------------------------

_pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=NEOPIXEL_BRIGHTNESS, auto_write=True)


def pixel_set(colour):
    """Set NeoPixel to an (r, g, b) tuple."""
    _pixel[0] = colour


PIXEL_OFF = (0, 0, 0)
PIXEL_GREEN = (0, 204, 68)
PIXEL_AMBER = (255, 170, 0)
PIXEL_RED = (255, 34, 34)
PIXEL_BLUE = (68, 136, 255)


def pixel_for_status(status):
    s = status.lower()
    if "no mqtt" in s or "fail" in s or "error" in s:
        return PIXEL_RED
    if "warm" in s or "publish" in s:
        return PIXEL_AMBER
    return PIXEL_GREEN


# ---------------------------------------------------------------------------
# Native WiFi (ESP32-S3 built-in radio)
# ---------------------------------------------------------------------------

def read_rssi():
    try:
        return wifi.radio.ap_info.rssi
    except Exception:
        return 0


def connect_wifi():
    draw_display(0, "WiFi connecting", show_prefix=False)
    print("Connecting to WiFi...")
    while not wifi.radio.connected:
        pat_watchdog()
        try:
            wifi.radio.connect(
                os.getenv("CIRCUITPY_WIFI_SSID"),
                os.getenv("CIRCUITPY_WIFI_PASSWORD"),
            )
        except Exception as exc:
            print("  WiFi error:", exc, "- retrying")
            time.sleep(2)
    ip = str(wifi.radio.ipv4_address)
    print("Connected  IP:", ip)
    draw_display(0, "WiFi: " + ip, show_prefix=False)
    time.sleep(1)


connect_wifi()

# ---------------------------------------------------------------------------
# NTP time sync — one-shot at boot; RTC holds time thereafter
# ---------------------------------------------------------------------------

_clock_valid = False


def sync_ntp():
    global _clock_valid
    try:
        ntp = adafruit_ntp.NTP(socketpool.SocketPool(wifi.radio), tz_offset=0)
        rtc.RTC().datetime = ntp.datetime
        _clock_valid = True
        print("NTP synced")
    except Exception as exc:
        print("NTP sync failed:", exc)


def clock_str():
    """Return HH:MM:SS string from RTC, or '--:--:--' if not synced."""
    if not _clock_valid:
        return "--:--:--"
    t = rtc.RTC().datetime
    return (str(t.tm_hour).zfill(2) + ":" +
            str(t.tm_min).zfill(2) + ":" +
            str(t.tm_sec).zfill(2))


sync_ntp()


# ---------------------------------------------------------------------------
# MQTT — command and config handlers
# ---------------------------------------------------------------------------

_pending_reboot = False
_pending_calibrate = False
_pending_nvm_reset = False
_pending_identify = False
_pending_diag = False
_pending_config = {}


def _on_cmd(client, topic, message):
    global _pending_reboot, _pending_calibrate
    global _pending_nvm_reset, _pending_identify, _pending_diag
    cmd = message.strip().lower()
    print("CMD received: " + cmd)
    if cmd == "reboot":
        _pending_reboot = True
    elif cmd == "calibrate":
        _pending_calibrate = True
    elif cmd == "nvm_reset":
        _pending_nvm_reset = True
    elif cmd == "identify":
        _pending_identify = True
    elif cmd == "status":
        _pending_diag = True
    else:
        print("Unknown command: " + cmd)


def _on_cfg(client, topic, message):
    param = topic.split("/")[-1]
    try:
        value = int(float(message.strip()))
    except ValueError:
        print("Config bad value for " + param + ": " + message)
        return
    _pending_config[param] = value
    print("CFG queued: " + param + "=" + str(value))


def apply_pending_config():
    global publish_interval, trend_threshold, ewma_n, ewma_alpha
    global last_publish
    if not _pending_config:
        return False
    changed = False
    if "publish_interval" in _pending_config:
        v = _pending_config.pop("publish_interval")
        v = max(PUBLISH_INTERVAL_MIN, min(v, PUBLISH_INTERVAL_MAX))
        if v != publish_interval:
            publish_interval = v
            last_publish = -publish_interval
            changed = True
            print("Config applied: publish_interval=" + str(publish_interval))
    if "trend_threshold" in _pending_config:
        v = _pending_config.pop("trend_threshold")
        v = max(TREND_THRESHOLD_MIN, min(v, TREND_THRESHOLD_MAX))
        if v != trend_threshold:
            trend_threshold = v
            changed = True
            print("Config applied: trend_threshold=" + str(trend_threshold))
    if "ewma_n" in _pending_config:
        v = _pending_config.pop("ewma_n")
        v = max(EWMA_N_MIN, min(v, EWMA_N_MAX))
        if v != ewma_n:
            ewma_n = v
            ewma_alpha = 2.0 / (ewma_n + 1)
            changed = True
            print("Config applied: ewma_n=" + str(ewma_n)
                  + "  alpha=" + "{:.4f}".format(ewma_alpha))
    _pending_config.clear()
    if changed:
        _nvm_write_config()
    return changed


# ---------------------------------------------------------------------------
# MQTT client — native ESP32-S3 socket pool
# ---------------------------------------------------------------------------

_pool = socketpool.SocketPool(wifi.radio)
_ssl_context = ssl.create_default_context()
mqtt_client = MQTT.MQTT(
    broker=os.getenv("MQTT_BROKER"),
    port=int(os.getenv("MQTT_PORT", 1883)),
    username=os.getenv("MQTT_USERNAME") or None,
    password=os.getenv("MQTT_PASSWORD") or None,
    client_id=os.getenv("MQTT_CLIENT", DEVICE_ID),
    socket_pool=_pool,
    ssl_context=_ssl_context,
    keep_alive=300,
    socket_timeout=1,
)
mqtt_client.will_set(AVAIL_TOPIC, "offline", retain=True)
mqtt_client.on_message = _on_cmd

discovery_sent = False


def do_publish(payload, extra_publish=None):
    """Open a fresh MQTT connection, publish readings, then disconnect."""
    global discovery_sent
    pat_watchdog()
    if not wifi.radio.connected:
        connect_wifi()
    print("MQTT connecting...")
    pat_watchdog()
    mqtt_client.connect()
    pat_watchdog()
    pat_watchdog()

    if not discovery_sent:
        print("Publishing discovery...")
        for topic, disc_payload in build_discovery_topics().items():
            mqtt_client.publish(topic, disc_payload, retain=True)
            pat_watchdog()
        discovery_sent = True

    mqtt_client.subscribe(CMD_TOPIC)
    mqtt_client.add_topic_callback(CMD_TOPIC, _on_cmd)
    mqtt_client.subscribe(CFG_TOPIC + "/#")
    mqtt_client.add_topic_callback(CFG_TOPIC + "/publish_interval",
                                   lambda c, t, m: _on_cfg(c, t, m))
    mqtt_client.add_topic_callback(CFG_TOPIC + "/trend_threshold",
                                   lambda c, t, m: _on_cfg(c, t, m))
    mqtt_client.add_topic_callback(CFG_TOPIC + "/ewma_n",
                                   lambda c, t, m: _on_cfg(c, t, m))
    pat_watchdog()

    mqtt_client.publish(AVAIL_TOPIC, "online", retain=True)
    pat_watchdog()
    mqtt_client.publish(STATE_TOPIC, payload, retain=True)
    pat_watchdog()

    if extra_publish:
        for ep_topic, ep_payload in extra_publish:
            mqtt_client.publish(ep_topic, ep_payload, retain=False)
            pat_watchdog()

    try:
        mqtt_client.loop(timeout=1)
        pat_watchdog()
    except Exception:
        pass

    mqtt_client.disconnect()
    pat_watchdog()
    print("MQTT published and disconnected")
    return "OK"


def build_diag_payload():
    raw = current_reading()
    diag = {
        "version": FIRMWARE_VERSION,
        "reset_reason": _RESET_REASON,
        "uptime_s": int(time.monotonic()),
        "raw": raw,
        "free_mem": gc.mem_free(),
        "rssi": read_rssi(),
        "obs_min": _obs_min,
        "obs_max": _obs_max,
        "publish_interval": publish_interval,
        "trend_threshold": trend_threshold,
        "ewma_n": ewma_n,
        "baseline_valid": _baseline_valid,
    }
    if _baseline_valid:
        diag["baseline"] = _baseline
    return json.dumps(diag)


# ---------------------------------------------------------------------------
# Boot publish
# ---------------------------------------------------------------------------

draw_display(0, "MQTT boot pub", show_prefix=False)
try:
    _boot_raw = sensor_pin.value
    _boot_payload = json.dumps({
        "raw": _boot_raw,
        "status": "Warming Up",
        "version": FIRMWARE_VERSION,
        "reset_reason": _RESET_REASON,
    })
    do_publish(_boot_payload)
    print("Boot publish  raw=" + str(_boot_raw)
          + "  reset_reason=" + _RESET_REASON)
except Exception as exc:
    print("Boot publish failed:", exc)


# ---------------------------------------------------------------------------
# Inter-core communication (IPC) — dual-core mode only
# ---------------------------------------------------------------------------

if _DUAL_CORE:
    _ipc_lock = _thread.allocate_lock()
    _ipc_request = False
    _ipc_payload = ""
    _ipc_extra = None
    _ipc_result = None

    def _mqtt_core_thread():
        global _ipc_request, _ipc_result
        while True:
            _ipc_lock.acquire()
            if _ipc_request:
                payload = _ipc_payload
                extra = _ipc_extra
                _ipc_request = False
                _ipc_lock.release()
                try:
                    result = do_publish(payload, extra)
                except Exception as exc:
                    print("Core1: publish failed:", exc)
                    result = "No MQTT"
                _ipc_lock.acquire()
                _ipc_result = result
                _ipc_lock.release()
            else:
                _ipc_lock.release()
                time.sleep(0.05)

    _thread.start_new_thread(_mqtt_core_thread, ())
    print("Dual-core: Core 1 started")
else:
    print("Single-core: _thread not available, publishing on Core 0")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

last_sample = -SAMPLE_INTERVAL
last_publish = -publish_interval
last_status = "Warming Up"
last_trend = "---"
last_hour_avg = None
last_rssi = 0
prev_raw = None
_cal_display_until = 0
_publish_in_flight = False

pixel_set(PIXEL_AMBER)   # amber during warmup

while True:
    pat_watchdog()
    now = time.monotonic()

    # --- Collect result from Core 1 (dual-core only) ---
    if _DUAL_CORE and _publish_in_flight:
        _ipc_lock.acquire()
        result = _ipc_result
        if result is not None:
            _ipc_result = None
            _publish_in_flight = False
        _ipc_lock.release()
        if result is not None:
            last_status = result
            pixel_set(pixel_for_status(last_status))

    # --- Apply any queued config changes ---
    apply_pending_config()

    # --- Handle pending commands ---
    if _pending_reboot:
        print("Remote reboot requested")
        draw_display(current_reading(), "Rebooting...", last_trend)
        time.sleep(1)
        microcontroller.reset()

    if _pending_nvm_reset:
        _pending_nvm_reset = False
        nvm_reset_all()
        draw_display(current_reading(), "NVM Reset", last_trend)

    if _pending_identify:
        _pending_identify = False
        identify_flash()

    if _pending_calibrate:
        _pending_calibrate = False
        if calibrate_baseline():
            _cal_display_until = now + CAL_MSG_DURATION
            print("Remote calibration successful")
        else:
            print("Remote calibration ignored: not warmed up")

    if _pending_diag:
        _pending_diag = False
        diag_payload = build_diag_payload()
        print("Diag publish: " + diag_payload)
        if _DUAL_CORE and not _publish_in_flight:
            _ipc_lock.acquire()
            _ipc_payload = json.dumps({"raw": current_reading(), "status": last_status})
            _ipc_extra = [(DIAG_TOPIC, diag_payload)]
            _ipc_request = True
            _ipc_result = None
            _ipc_lock.release()
            _publish_in_flight = True
        elif not _DUAL_CORE:
            try:
                do_publish(
                    json.dumps({"raw": current_reading(), "status": last_status}),
                    [(DIAG_TOPIC, diag_payload)],
                )
            except Exception as exc:
                print("Diag publish failed:", exc)

    # --- Sample every SAMPLE_INTERVAL seconds ---
    if now - last_sample >= SAMPLE_INTERVAL:
        last_sample = now
        sample_sensor()
        raw = current_reading()

        if now < _cal_display_until:
            status = "Calibrated"
        elif warmed_up():
            status = last_status
        else:
            remaining = (warmup_samples - _sample_count) * SAMPLE_INTERVAL
            status = "Warmup " + str(remaining) + "s"

        draw_display(raw, status, last_trend,
                     hour_avg=last_hour_avg,
                     rssi=last_rssi,
                     next_secs=(publish_interval - (now - last_publish)) if warmed_up() else None,
                     clock_str=clock_str())

        if warmed_up():
            delta_str = ("  delta=" + str(raw - _baseline)) if _baseline_valid else ""
            print("raw=" + str(raw)
                  + "  lo=" + str(_obs_min) + "  hi=" + str(_obs_max)
                  + "  status=" + last_status + "  trend=" + last_trend
                  + delta_str)
        else:
            print("Warming up  n=" + str(_sample_count) + "/" + str(warmup_samples)
                  + "  raw=" + str(raw))

    # --- Publish every publish_interval seconds, after warmup ---
    publish_due = warmed_up() and now - last_publish >= publish_interval
    if _DUAL_CORE:
        publish_due = publish_due and not _publish_in_flight

    if publish_due:
        last_publish = now
        raw = current_reading()
        last_trend = trend_symbol(prev_raw, raw) if prev_raw is not None else "Stable"
        prev_raw = raw
        record_hourly(raw)
        stats = hourly_stats()
        if stats is not None:
            last_hour_avg = stats[0]
        payload_dict = {
            "raw": raw,
            "trend": last_trend,
            "status": "OK",
            "version": FIRMWARE_VERSION,
            "reset_reason": _RESET_REASON,
        }
        if _baseline_valid:
            payload_dict["delta"] = raw - _baseline
        if stats is not None:
            h_avg, h_min, h_max = stats
            payload_dict["hour_avg"] = h_avg
            payload_dict["hour_min"] = h_min
            payload_dict["hour_max"] = h_max
            print("Hourly  avg=" + str(h_avg)
                  + "  min=" + str(h_min) + "  max=" + str(h_max))
        if _nvm_dirty:
            _nvm_write_sensor()
            pat_watchdog()
        rssi = read_rssi()
        last_rssi = rssi
        payload_dict["rssi"] = rssi
        if _DUAL_CORE:
            _ipc_lock.acquire()
            _ipc_payload = json.dumps(payload_dict)
            _ipc_extra = None
            _ipc_request = True
            _ipc_result = None
            _ipc_lock.release()
            _publish_in_flight = True
            last_status = "Publishing"
            pixel_set(PIXEL_BLUE)
            print("Publish queued  raw=" + str(raw) + "  trend=" + last_trend
                  + "  rssi=" + str(rssi) + " dBm")
        else:
            try:
                last_status = do_publish(json.dumps(payload_dict))
                pixel_set(pixel_for_status(last_status))
                print("Published  raw=" + str(raw) + "  trend=" + last_trend
                      + "  rssi=" + str(rssi) + " dBm")
            except Exception as exc:
                print("Publish failed:", exc)
                last_status = "No MQTT"
                pixel_set(PIXEL_RED)
            draw_display(raw, last_status, last_trend,
                         hour_avg=last_hour_avg, rssi=last_rssi,
                         next_secs=publish_interval, clock_str=clock_str())

    time.sleep(0.1)
