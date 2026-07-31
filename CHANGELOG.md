# Changelog

All notable changes to this component are documented here. Version numbers refer
to this repository, not to the upstream pull request.

## Unreleased

### Changed

* Update to the official post-release `esp_video` 2.3.0 commit that accepts any
  compatible `esp_h264` 1.3.x dependency.
* Use the renamed 2.3.0 hardware JPEG encoder Kconfig symbols.
* Keep direct JPEG/MJPEG builds free of the inactive `esp_h264` dependency
  through an empty local component stub; raw CSI integrations still resolve the
  real managed codec.

## v0.1.0 — 2026-07-27

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
