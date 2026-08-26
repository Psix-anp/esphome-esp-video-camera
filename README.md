# esp_video_camera — ESP32-P4 MIPI-CSI camera platform for ESPHome

An ESPHome external component that turns an ESP32-P4 with a MIPI-CSI sensor into
a **native ESPHome `camera` entity**: Home Assistant discovers it as a real
camera, snapshots work, and `web_server` serves an MJPEG stream — no go2rtc, no
external RTSP bridge, no LVGL display required.

Frames are produced entirely in hardware: the MIPI-CSI sensor feeds the P4 ISP
(RGB565), and the P4's **hardware JPEG encoder** (a V4L2 M2M device) compresses
them. A USB-UVC camera that already emits MJPEG can be used as an alternative
source.

Everything runs on Espressif's `esp_video` (V4L2) stack, pulled at build time
through the IDF component manager — nothing is vendored.

## Credit

This component is **not mine**. It was written by
[youkorr](https://github.com/youkorr) and submitted to ESPHome as
[pull request esphome/esphome#16944](https://github.com/esphome/esphome/pull/16944)
("[esp_video_camera] Add ESP32-P4 esp_video (V4L2) camera platform"), itself a
port of [youkorr/esphome_esp-video](https://github.com/youkorr/esphome_esp-video).
That PR is still open and unmerged.

This repository is that component **plus the fixes needed to make it actually
run on real hardware** (Waveshare ESP32-P4 + OV5647). The changes are documented
below; see [`NOTICE`](NOTICE) for licensing and attribution.

---

## Fixes on top of the original PR

This is the substance of this fork. On the original PR the component failed at
boot, and once past that it hard-locked the board. Six changes:

### 1. JPEG CAPTURE `S_FMT` must carry a non-zero resolution

`esp_video` 2.2.0 validates `width`/`height` on **both** sides of the JPEG M2M
device, not just the OUTPUT side. In `jpeg_video_set_format()`, a `width < MIN`
or `height < MIN` returns `EINVAL`. The original code zeroed the `v4l2_format`
struct for the CAPTURE side and only set `pixelformat = V4L2_PIX_FMT_JPEG`,
i.e. it asked for a 0x0 JPEG — so every boot ended in:

```
JPEG CAPTURE S_FMT failed: Invalid argument
```

Fix: propagate the negotiated capture resolution to the CAPTURE format too
(`esp_video_camera.cpp`, `start_jpeg_pipeline_()`).

### 2. Capture moved into its own FreeRTOS task

The original ran the whole capture cycle — including **blocking** `VIDIOC_DQBUF`
ioctls on the JPEG M2M fd — directly inside ESPHome's `loop()`. Any stall over
~5 s trips the task watchdog on `loopTask` and reboots the board.

This is not a corner case: Home Assistant polls the camera entity by itself, so
the board went into a reboot loop roughly every 40 s with no user interaction.

Fix: a dedicated task (`esp_video_cap`, 8 KB internal stack, priority 3, pinned
to CPU0 — `loopTask` runs on CPU1) owns the pipeline and all blocking ioctls. It
copies the finished JPEG into PSRAM and parks it in a mutex-protected pending
slot; `loop()` only picks the frame up and hands it to the listeners, because
the ESPHome API callbacks are not thread-safe. This mirrors what the core
`esp32_camera` component does.

### 3. `DQBUF` order: CAPTURE first, then OUTPUT

In `esp_video` 2.2.0 the hardware encode is **lazy**. `esp_video_recv_element()`
notifies `M2M_TRIGGER` (which is what actually starts `m2m_process`) only for
`type == V4L2_BUF_TYPE_VIDEO_CAPTURE` — see `jpeg_video_notify()`. The original
code dequeued the OUTPUT buffer first. That never kicks the encoder off, so the
call blocks on `ready_sem` forever. This was the real source of the watchdog
hang in (2); moving capture to a task only turned a reboot into a silent stall.

Fix: `DQBUF(CAPTURE)` first (starts the encode and waits for it), then
`DQBUF(OUTPUT)` to reclaim the input buffer — which returns immediately, since
by then the encoder is done with it.

### 4. Runtime controls go through `VIDIOC_S_EXT_CTRLS`, never `VIDIOC_S_CTRL`

`esp_video`'s ioctl table implements only the extended-control interface. The
legacy `VIDIOC_S_CTRL` returns `EINVAL`. That is why the `jpeg_quality:` option
from the original PR **never took effect**: it is written once with
`VIDIOC_S_CTRL` in `start_jpeg_pipeline_()` and the return value is not checked,
so the encoder silently kept its default quality.

Fix: every control write — the static `jpeg_quality:` option and the runtime
controls alike — goes through `VIDIOC_S_EXT_CTRLS` with a properly filled
`v4l2_ext_controls` (`v4l2_set_ext_ctrl()`), and the result is logged. This is
also what made the runtime controls below possible.

There is also a one-shot enumeration of the sensor/ISP controls
(`VIDIOC_QUERY_EXT_CTRL` with `V4L2_CTRL_FLAG_NEXT_CTRL`) logged on the first
successful capture start, so you can see what your sensor actually supports:

```
[esp_video_camera] sensor ctrl: 0x00980911 'Exposure' type=1 min=2 max=235 step=1 def=... flags=0x0
[esp_video_camera] sensor ctrl: 12 controls enumerated
```

### 5. Warmup: unstable startup frames are withheld

The AE/IPA loop starts cold. With an event-driven use case (capture a snapshot
when someone presses a button) the very first frame off a freshly started
pipeline is essentially black.

The hardware JPEG path discards the first two CSI buffers after every
`STREAMON`, matching Espressif's V4L2 examples. The raw CSI path retains a
250 ms deadline because hardware frame skipping changes how often userspace
receives a frame.

### 6. Linger: the pipeline stays alive 5 s after the last request

Tearing the pipeline down the moment the last requester disappears means the
next snapshot pays the warmup cost again. In practice events arrive in bursts.

Fix: `LINGER_MS = 5000`. A one-shot ESPHome timeout clears capture intent after
5 s; the capture task then stops the V4L2 queues while retaining the prepared
descriptors and mapped buffers. A later request requeues the same buffers and
restarts streaming without reallocating them or polling the main loop.

---

## Requirements

* ESP32-P4 with a MIPI-CSI sensor supported by `esp_cam_sensor`
  (auto-detected: SC202CS, OV5647, SC2336), or a USB-UVC camera.
* External PSRAM for the JPEG handoff copy.
* ESP-IDF **≥ 5.4** (required by `esp_video` 2.4.1).
* **The `esp-idf` toolchain — this is mandatory**, see below.

### `esp32: toolchain: esp-idf` is required

The component pulls managed Espressif components (`add_idf_component()`) and
sets Kconfig options (`add_idf_sdkconfig_option()`). That machinery only works
on the native ESP-IDF toolchain. On the PlatformIO toolchain `esphome config`
still validates, and the build then fails during code generation with:

```
esp_video_camera requires the esp-idf framework.
```

* **ESPHome 2026.7.0 and later** — `esp-idf` is the default toolchain; nothing
  to do.
* **ESPHome 2026.6.x** — the option exists but defaults to `platformio`. Set it
  explicitly:

```yaml
esp32:
  variant: esp32p4
  toolchain: esp-idf   # REQUIRED on 2026.6.x — default there is platformio
  framework:
    type: esp-idf
```

Note that the first build with this toolchain downloads the full IDF toolchain
(a few minutes and several GB into `.esphome/idf/`).

---

## Configuration

```yaml
external_components:
  # This component.
  - source: github://Psix-anp/esphome-esp-video-camera
    components: [esp_video_camera]
  # The base `camera` platform is not in ESPHome yet either — it comes from the
  # same (still unmerged) pull request.
  - source: github://pr#16944
    components: [camera]
    refresh: 1d

esp32:
  variant: esp32p4
  toolchain: esp-idf
  framework:
    type: esp-idf

# SCCB (sensor control) rides on a normal ESPHome I2C bus — no hardcoded pins.
i2c:
  - id: cam_i2c
    sda: GPIO7
    scl: GPIO8
    frequency: 400kHz

esp_video_camera:
  id: my_camera
  name: "Camera"
  i2c_id: cam_i2c
  device: jpeg        # hardware JPEG encoder (/dev/video10) — MIPI-CSI sensors
  resolution: "auto"  # "auto" = the sensor's native/default mode
  jpeg_quality: 10    # 1..63
  rotation: 0         # 0/90/180/270 degrees clockwise, hardware PPA
  max_framerate: 10
  enable_xclk: false  # true only if the sensor needs an XCLK from the MCU
```

### Options

| Option | Default | Notes |
| --- | --- | --- |
| `i2c_id` | *required* | ESPHome I2C bus used as SCCB. Can be shared with other devices. |
| `device` | `jpeg` | `jpeg` = MIPI-CSI + hardware JPEG (`/dev/video10`); `csi` = raw CSI device; `uvc` / `uvc0`..`uvc9` = USB-UVC; or an explicit `/dev/videoN`. |
| `resolution` | `auto` | `auto`, an alias (`QVGA`/`VGA`/`480P`/`720P`/`1080P`) or `WIDTHxHEIGHT`. Best-effort: the sensor must actually support the mode, see "Sensor modes". |
| `jpeg_quality` | `10` | 1..63 (`V4L2_CID_JPEG_COMPRESSION_QUALITY`). |
| `rotation` | `0` | `0`, `90`, `180` or `270` degrees clockwise. Hardware PPA; MIPI-CSI JPEG path only. |
| `max_framerate` | `10` | Programs V4L2 frame skipping when supported. If the source cannot apply it, ESPHome API frames use a software throttle while borrowed consumers retain the sensor frame rate. |
| `enable_xclk` | `false` | Generate the sensor XCLK with LEDC before init. Not needed for modules with their own crystal. |
| `xclk_pin` | `36` | Only used when `enable_xclk: true`. |
| `xclk_frequency` | `24000000` | Only used when `enable_xclk: true`. |
| `enable_uvc` | `false` | Pull in the USB-UVC host driver (`usb_host_uvc`). |

`reset_pin` / `pwdn_pin` are hardcoded to `-1`; boards that need a reset are
expected to hold it via hardware (e.g. a pull-up on the CSI connector).

When a sensor coerces an explicit `resolution:` to a larger native mode, the
same PPA transaction can downscale before JPEG encoding if the ratio is exactly
representable at the hardware's 1/16 scale precision. Otherwise the negotiated
sensor size is retained and a warning is logged. Rotation and downscaling stay
in RGB565 and do not add a software decode/re-encode step.

---

## Runtime controls

Beyond the static YAML options, five settings can be changed at runtime from
lambdas — so they can be exposed to Home Assistant as `number` / `switch`
entities. The setters only store the value into an atomic field; the capture
task applies it between frames on the live file descriptors, so no ioctl is ever
issued from a foreign thread.

| Setter | Control | Range |
| --- | --- | --- |
| `set_runtime_exposure(int)` | `V4L2_CID_EXPOSURE` | sensor-specific (2..235 on the OV5647) |
| `set_runtime_vflip(bool)` | `V4L2_CID_VFLIP` | on/off |
| `set_runtime_hflip(bool)` | `V4L2_CID_HFLIP` | on/off |
| `set_runtime_jpeg_quality(int)` | `V4L2_CID_JPEG_COMPRESSION_QUALITY` | 1..63 |
| `set_runtime_max_fps(float)` | current-session publish throttle; V4L2 rate is reapplied on the next capture start | > 0 |

Check the `sensor ctrl:` enumeration in the log (fix 4) to see which controls
your sensor really implements. Exposure in particular may be partially
overridden by the ISP's automatic AE.

```yaml
switch:
  - platform: template
    name: "Camera Flip Vertical"
    icon: mdi:flip-vertical
    entity_category: config
    optimistic: true
    restore_mode: RESTORE_DEFAULT_OFF
    turn_on_action:
      - lambda: 'id(my_camera).set_runtime_vflip(true);'
    turn_off_action:
      - lambda: 'id(my_camera).set_runtime_vflip(false);'

number:
  - platform: template
    name: "Camera Exposure"
    icon: mdi:camera-iris
    entity_category: config
    min_value: 2
    max_value: 235
    step: 1
    initial_value: 80
    restore_value: true
    optimistic: true
    on_value:
      - lambda: 'id(my_camera).set_runtime_exposure((int) x);'

  - platform: template
    name: "Camera JPEG Quality"
    entity_category: config
    min_value: 1
    max_value: 63
    step: 1
    initial_value: 10
    restore_value: true
    optimistic: true
    on_value:
      - lambda: 'id(my_camera).set_runtime_jpeg_quality((int) x);'

  - platform: template
    name: "Camera Max FPS"
    entity_category: config
    min_value: 1
    max_value: 30
    step: 1
    initial_value: 10
    restore_value: true
    optimistic: true
    on_value:
      - lambda: 'id(my_camera).set_runtime_max_fps(x);'
```

Single frames can also be grabbed from a lambda, which is what the warmup and
linger fixes are for:

```yaml
    - lambda: |-
        if (!id(my_camera).is_failed())
          id(my_camera).request_image(esphome::camera::IDLE);
```

---

## Sensor modes

`resolution:` is best-effort — `VIDIOC_S_FMT` can only pick from the modes the
sensor driver was **compiled** with. Which modes exist is a Kconfig decision in
`esp_cam_sensor`, so unusual resolutions are selected through
`sdkconfig_options`, not through YAML.

For the **OV5647** the available MIPI 2-lane modes (24 MHz input) are:

| Mode | Kconfig symbol suffix | Enabled by default |
| --- | --- | --- |
| 800x640 @50 fps, RAW8 | `RAW8_800X640_50FPS` | no |
| **800x800 @50 fps, RAW8** | `RAW8_800X800_50FPS` | **yes (default mode)** |
| 800x1280 @50 fps, RAW8 | `RAW8_800X1280_50FPS` | no |
| 1920x1080 @30 fps, RAW10 | `RAW10_1920X1080_30FPS` | no |
| 1280x960 binning @45 fps, RAW10 | `RAW10_1280X960_BINNING_45FPS` | no |

There is **no 1280x720 mode** — `resolution: 720P` will silently fall back to
whatever the sensor negotiates. With `resolution: "auto"` you get 800x800.

To use the 1280x960 2x2-binning mode (better low-light sensitivity, which is why
it is worth the trouble) enable it *and* make it the default format:

```yaml
esp32:
  framework:
    sdkconfig_options:
      CONFIG_CAMERA_OV5647_MIPI_RAW10_1280X960_BINNING_45FPS: "y"
      CONFIG_CAMERA_OV5647_MIPI_DEFAULT_FMT_RAW10_1280X960_BINNING_45FPS: "y"
```

Changing `sdkconfig_options` forces a full rebuild. The component's
`resolution: "auto"` then picks up that default format. The actually negotiated
resolution is read back and logged at startup:

```
[esp_video_camera] Capture resolution: 1280x960
```

The same pattern applies to the other sensors — check
`managed_components/espressif__esp_cam_sensor/sensors/<sensor>/Kconfig.<sensor>`
in your build directory for the exact symbols.

---

## Borrowed frame consumers

Native media components can register one raw-frame consumer and one JPEG-frame
consumer without opening the sensor or V4L2 queues a second time.

* `RawVideoFrameConsumer` receives RGB565 frames directly from the MIPI-CSI/ISP
  queue, before hardware JPEG encoding.
* `JpegFrameConsumer` receives the JPEG access unit produced by the hardware
  encoder, or the MJPEG frame supplied by a direct UVC source.
* `device: csi` is a raw-consumer-only mode. It does not publish images through
  the ESPHome camera API because no JPEG frame is produced.

Both callbacks execute synchronously on the capture task. Their data pointers
are borrowed and valid **only for the duration of the callback**. A consumer
must copy the payload before returning if it needs to retain or queue the frame,
and should keep callback work bounded so the V4L2 buffer can be re-queued
promptly.

`stop_raw_frame_consumer()` and `stop_jpeg_frame_consumer()` prevent future
callback delivery but do not wait for a callback already running on the capture
task. The consumer object and any callback-owned state must remain valid until
that in-flight callback has returned.

`max_framerate` throttles only frames copied to the ESPHome camera API. Borrowed
consumers receive every sensor frame. A JPEG consumer therefore keeps the
hardware encoder running at the sensor rate even when no Home Assistant camera
listener is active.

Registration and activation are separate: register a stable consumer object
once, then use `start_raw_frame_consumer()` / `stop_raw_frame_consumer()` or
their JPEG equivalents to control delivery. At most one consumer of each type
can be registered.

---

## Tested on

* **Board:** Waveshare ESP32-P4-WIFI6-PoE-ETH
* **Sensor:** OV5647 module on the 15-pin CSI connector
* **ESPHome:** 2026.6.2 (with `esp32: toolchain: esp-idf`)
* **ESP-IDF:** 5.5.4
* **Hardware-qualified baseline:** `espressif/esp_video` 2.2.0 with
  `esp_cam_sensor` 2.2.x and `esp_ipa` 2.1.x.

Board-specific notes, which generalise reasonably well:

* **SCCB shares the general-purpose I2C bus.** On this board the CSI connector's
  `ESP_I2C_SDA`/`ESP_I2C_SCL` are GPIO7/GPIO8 — the same bus as the audio codec.
  The OV5647 answers at SCCB address 0x36 and does not collide with the ES8311
  codec at 0x18. One `i2c:` bus, passed via `i2c_id`, serves both.
* **No XCLK line on the CSI connector** — the OV5647 module is clocked by its own
  crystal, so `enable_xclk: false` and no `xclk_pin` is needed. Boards that route
  MCLK to the sensor need `enable_xclk: true`.
* **Reset is done in hardware** — `CSI_IO0` sits on a 10 K pull-up to 3V3, so the
  hardcoded `reset_pin = -1` is fine.

Verified working: sensor detected over SCCB, IPA tuning loaded, ISP streaming
(AE converging), JPEG frames delivered over the ESPHome API (e.g. 800x800 →
~21.7 KB), camera entity visible in Home Assistant, no watchdog resets.

The current dependency graph uses the published `esp_video` 2.4.1 component,
`esp_cam_sensor` 2.4.x, `esp_ipa` 2.3.x and compatible `esp_h264` 1.3.x.

---

## Known limitations

* **The upstream PR is not merged.** Both this component and the base `camera`
  platform it builds on come from
  [esphome/esphome#16944](https://github.com/esphome/esphome/pull/16944), which
  is open and may change or be rejected. Nothing here is API-stable.
* **The ESP32-P4 has a single hardware JPEG engine**, shared between encode and
  decode. If something else in your firmware decodes JPEG in hardware (an image
  component, a display pipeline), the two contend for the same device.
* **JPEG/MJPEG output only.** There is no H.264 path in this camera component.
  Direct JPEG/MJPEG builds satisfy esp_video's inactive H.264 manifest edge
  with an empty local component, so `esp_h264` is neither downloaded nor
  compiled. Raw CSI integrations do not install that stub and can resolve the
  real codec from their separate encoder component.
* **`resolution:` cannot conjure modes** the sensor driver was not compiled with
  — see "Sensor modes".
* **Only three MIPI sensors are auto-detected** (SC202CS, OV5647, SC2336); that
  list is hardcoded in `__init__.py`.
* **`jpeg_quality:` only applies to the hardware encoder** on the `device: jpeg`
  path, not to a UVC source that hands you pre-encoded MJPEG. (It reaches the
  encoder since the extended-control fix — see fix 4; `set_runtime_jpeg_quality()`
  overrides it at runtime.)
* The USB-UVC path is inherited from the original PR and is **not tested here**.

## License

ESPHome License (GPLv3 for C++/runtime sources, MIT for the Python sources and
everything else) — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).
