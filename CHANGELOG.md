# Changelog

All notable changes to this component are documented here. Version numbers refer
to this repository, not to the upstream pull request.

## Unreleased

### Added

* Synchronous borrowed-frame consumers for native RGB565 CSI frames and encoded
  JPEG frames. Consumers can start and stop capture independently from Home
  Assistant camera requesters without opening the sensor a second time.
* `device: csi` now operates as a raw-consumer-only pipeline and bypasses the
  hardware JPEG encoder.
* Hardware PPA rotation for the MIPI-CSI JPEG path, configured as 0, 90, 180
  or 270 degrees clockwise.
* Hardware downscaling before JPEG encoding when a requested resolution is
  coerced upward by the sensor and is exactly representable at 1/16 precision.

### Changed

* Update to the published `esp_video` 2.4.1 component, with `esp_cam_sensor`
  2.4.x, `esp_ipa` 2.3.x and compatible `esp_h264` 1.3.x dependencies.
* Use the renamed 2.3.0 hardware JPEG encoder Kconfig symbols.
* Keep direct JPEG/MJPEG builds free of the inactive `esp_h264` dependency
  through an empty local component stub; raw CSI integrations still resolve the
  real managed codec.
* Capture now blocks on V4L2 frame events with a bounded driver timeout instead
  of polling a non-blocking descriptor with a one-tick delay.
* Hardware JPEG warmup discards the first two CSI buffers after `STREAMON`;
  raw CSI retains a 250 ms deadline. Linger and recovery use one-shot ESPHome
  timers.
* `max_framerate` uses `VIDIOC_S_PARM` hardware frame skipping when supported,
  with the existing software throttle retained as a fallback.
* Per-frame ISP/IPA debug telemetry is clamped to warnings through ESP-IDF's
  linked-list tag-level filter.

### Fixed

* Treat an explicit `/dev/video0` source as the same raw CSI mode as
  `device: csi`, matching the Python configuration and build-time dependency
  selection.

* Add bounded capture restart attempts after V4L2 failures.
* Add clean capture-task and `esp_video` shutdown.
* Keep prepared V4L2 buffers across idle linger cycles and resume them without
  per-session allocation.
* Preserve one-shot requester ownership when the pending-frame slot is
  overwritten and make cross-task requester state atomic.
* Stop reusing a raw USERPTR after the JPEG device has accepted ownership.

## v0.1.0, 2026-07-27

First published version. Baseline is the `esp_video_camera` component from
[esphome/esphome#16944](https://github.com/esphome/esphome/pull/16944) by
[youkorr](https://github.com/youkorr), as of the state fetched 2026-07-25.

All changes are confined to `esp_video_camera.cpp` and `esp_video_camera.h` and
are marked with `// FORK:` comments in the source. `__init__.py`, `i2c_helper.h`
and `cfg/sc202cs.json` are unchanged from the pull request.

### Fixed

* **JPEG CAPTURE `S_FMT` sent a 0x0 resolution.** `esp_video` 2.2.0 validates
  `width`/`height` on the CAPTURE side of the JPEG M2M device too
  (`jpeg_video_set_format()`: `width < MIN || height < MIN` → `EINVAL`), so the
  component failed at boot with `JPEG CAPTURE S_FMT failed: Invalid argument`.
  The negotiated capture resolution is now propagated to the CAPTURE format.
* **Blocking capture in `loop()` tripped the task watchdog.** The blocking
  `VIDIOC_DQBUF` ioctls ran on `loopTask`; a stall over ~5 s rebooted the board,
  and since Home Assistant polls the camera entity on its own this became a
  crash loop roughly every 40 s. Capture now runs in a dedicated FreeRTOS task
  (`esp_video_cap`, 8 KB internal stack, priority 3, pinned to CPU0). The task
  copies the finished JPEG into PSRAM and parks it in a mutex-protected pending
  slot; `loop()` only hands frames to the listeners, because the ESPHome API
  callbacks are not thread-safe. Same pattern as the core `esp32_camera`
  component.
* **`DQBUF` order on the JPEG M2M device.** In `esp_video` 2.2.0 the encode is
  lazy: `esp_video_recv_element()` notifies `M2M_TRIGGER` only for
  `type == V4L2_BUF_TYPE_VIDEO_CAPTURE` (`jpeg_video_notify()`). The original
  dequeued OUTPUT first, which never starts the encode and blocks on
  `ready_sem` forever — the actual root cause of the watchdog hang. Order is now
  `DQBUF(CAPTURE)` (starts and awaits the encode), then `DQBUF(OUTPUT)`.
* **Control writes used the unsupported `VIDIOC_S_CTRL`.** `esp_video`
  implements only the extended-control interface; the legacy ioctl returns
  `EINVAL`, which is why the static `jpeg_quality:` option never reached the
  encoder. Every control write — the static option and the runtime controls
  alike — now uses `VIDIOC_S_EXT_CTRLS` with a properly filled
  `v4l2_ext_controls`, and logs its result.
* **Black snapshots right after a start.** The AE/IPA loop needs about 10 frames
  to converge, so the first frame off a freshly started pipeline was essentially
  black. The first `WARMUP_FRAMES = 10` sensor frames (counted before the
  `max_framerate` throttle) are now dequeued and discarded.

### Added

* **Linger.** The capture pipeline now stays alive `LINGER_MS = 5000` after the
  last request instead of being torn down immediately, so a burst of events is
  served by an already warm camera without repeating warmup.
* **Runtime controls**, applied by the capture task between frames on the live
  file descriptors (never from a foreign thread): `set_runtime_exposure()`,
  `set_runtime_vflip()`, `set_runtime_hflip()`, `set_runtime_jpeg_quality()`,
  `set_runtime_max_fps()`. Intended to be wired to `number` / `switch` template
  entities — see README.
* **One-shot V4L2 control enumeration** into the log on the first successful
  capture start (`VIDIOC_QUERY_EXT_CTRL` with `V4L2_CTRL_FLAG_NEXT_CTRL`), so
  the controls a given sensor actually supports are discoverable.
* **DQBUF diagnostics** for the first three frames (`dbg:` log markers), which
  is how the lazy-encode ordering bug was pinned down.

### Notes

* Source comments in the C++ files were translated from Russian to English for
  publication. Verified comment-only: with all comments stripped, the sources
  are byte-identical to the versions running on the author's hardware.
* Requires the ESP-IDF toolchain (`esp32: toolchain: esp-idf`). On ESPHome
  2026.6.x this must be set explicitly; from 2026.7.0 it is the default.
* Verified on a Waveshare ESP32-P4-WIFI6-PoE-ETH with an OV5647, ESPHome 2026.6.2,
  ESP-IDF 5.5.4, `espressif/esp_video` 2.2.0.
