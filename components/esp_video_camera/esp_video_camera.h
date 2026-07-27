#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP_IDF

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/i2c/i2c.h"

#include "driver/gpio.h"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace esphome::esp_video_camera {

/// An owned JPEG/MJPEG frame (copied into PSRAM) shared with the API.
///
/// The data is JPEG-encoded (required by the Home Assistant camera API). It is
/// copied out of the mapped V4L2 buffer so that buffer can be re-queued
/// immediately, while the API streams this copy out over the network.
class ESPVideoCameraImage : public camera::CameraImage {
 public:
  ESPVideoCameraImage(uint8_t *data, size_t length, uint8_t requesters);
  ~ESPVideoCameraImage() override;

  uint8_t *get_data_buffer() override { return this->data_; }
  size_t get_data_length() override { return this->length_; }
  bool was_requested_by(camera::CameraRequester requester) const override;

 protected:
  uint8_t *data_{nullptr};
  size_t length_{0};
  uint8_t requesters_{0};
};

/// Reader used by the API to stream the JPEG bytes out in chunks.
class ESPVideoCameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<camera::CameraImage> image_;
  size_t offset_{0};
};

/// Home Assistant camera backed by Espressif's esp_video (V4L2) pipeline.
///
/// This single component both initialises the camera pipeline (MIPI-CSI, with an
/// optional USB-UVC host) and publishes the stream as a native `camera` entity.
/// It captures JPEG/MJPEG frames from a V4L2 device:
///   - "jpeg": the hardware JPEG encoder (/dev/video10) — works with every
///     auto-detected MIPI-CSI sensor (SC202CS, OV5647, SC2336, ...).
///   - "uvc":  a USB-UVC camera (/dev/video40+) that streams MJPEG.
///   - "/dev/videoN": an explicit V4L2 path.
class ESPVideoCamera : public camera::Camera {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Pipeline configuration -----------------------------------------------------
  void set_i2c_bus(i2c::I2CBus *bus) { this->i2c_bus_ = bus; }
  void set_xclk_pin(gpio_num_t pin) { this->xclk_pin_ = pin; }
  void set_xclk_freq(uint32_t freq) { this->xclk_freq_ = freq; }
  void set_enable_xclk_init(bool enable) { this->enable_xclk_init_ = enable; }
  void set_enable_uvc(bool enable) { this->enable_uvc_ = enable; }

  // Camera platform configuration ----------------------------------------------
  void set_device(const std::string &device) { this->device_ = device; }
  void set_resolution(const std::string &resolution) { this->resolution_ = resolution; }
  void set_jpeg_quality(int quality) { this->jpeg_quality_ = quality; }
  void set_max_framerate(float fps) {
    this->max_framerate_ = fps;
    this->min_interval_ms_ = (fps > 0.0f) ? (uint32_t) (1000.0f / fps) : 0;
  }

  // FORK: runtime settings coming from Home Assistant / the web server. The
  // values live in atomic fields and are applied by the CAPTURE TASK
  // (apply_runtime_ctrls_) on the live file descriptors — no ioctl is ever
  // issued from a foreign thread. -1 means "unset" (driver default / auto).
  void set_runtime_exposure(int v) {
    this->rt_exposure_.store(v);
    this->ctrls_dirty_.store(true);
  }
  void set_runtime_vflip(bool v) {
    this->rt_vflip_.store(v ? 1 : 0);
    this->ctrls_dirty_.store(true);
  }
  void set_runtime_hflip(bool v) {
    this->rt_hflip_.store(v ? 1 : 0);
    this->ctrls_dirty_.store(true);
  }
  void set_runtime_jpeg_quality(int q) {
    this->rt_quality_.store(q);
    this->ctrls_dirty_.store(true);
  }
  void set_runtime_max_fps(float fps) {
    this->min_interval_ms_ = (fps > 0.0f) ? (uint32_t) (1000.0f / fps) : 0;
  }

  // camera::Camera -------------------------------------------------------------
  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override;
  void request_image(camera::CameraRequester requester) override;
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

 protected:
  bool init_pipeline_();
  bool start_capture_();
  void stop_capture_();
  void update_capture_state_();

  // FORK: capture moved out of loop() into a dedicated FreeRTOS task. The
  // blocking DQBUF ioctls stalled loopTask for >5 s → task_wdt → reboot
  // (a crash loop, because Home Assistant polls the camera entity on its own).
  // Same pattern as the core esp32_camera component: the task captures and
  // parks the finished JPEG in pending_jpeg_ (under a mutex); loop() only
  // picks it up and hands it to the listeners (the API is not thread-safe).
  static void capture_task_trampoline(void *param);
  void capture_task_run_();
  // From the task: copy the finished JPEG into PSRAM and park it in the slot.
  void queue_frame_(const uint8_t *data, size_t length);
  // From loop(): hand the copy to the listeners (ownership moves to the image).
  void deliver_frame_owned_(uint8_t *data, size_t length);
  // FORK: one-shot enumeration of the sensor/ISP V4L2 controls into the log —
  // shows what the hardware actually supports (flip/brightness/exposure/...)
  // so that runtime settings can be wired up against real controls.
  void log_sensor_controls_();
  // Apply the rt_* settings on the live fds (called ONLY from the capture task).
  void apply_runtime_ctrls_();
  bool configure_capture_format_(uint32_t pixelformat);
  bool setup_capture_buffers_();
  // Hardware-JPEG path: capture RGB565 (sensor/ISP) -> JPEG M2M encoder.
  bool start_jpeg_pipeline_();
  void loop_jpeg_pipeline_();
  // Direct path: a source that already delivers JPEG/MJPEG (USB-UVC / device).
  bool start_direct_capture_();
  void loop_direct_capture_();

  // Pipeline
  i2c::I2CBus *i2c_bus_{nullptr};
  gpio_num_t xclk_pin_{GPIO_NUM_36};
  uint32_t xclk_freq_{24000000};
  bool enable_xclk_init_{false};
  bool enable_uvc_{false};
  bool pipeline_ready_{false};

  // Camera platform
  std::string device_{"jpeg"};
  std::string resolved_device_;
  bool is_hw_jpeg_{false};
  std::string resolution_{"auto"};
  int jpeg_quality_{10};
  float max_framerate_{10.0f};
  uint32_t min_interval_ms_{100};
  uint32_t last_frame_ms_{0};

  // Consumers (bit masks indexed by camera::CameraRequester)
  std::vector<camera::CameraListener *> listeners_;
  std::shared_ptr<ESPVideoCameraImage> current_image_;
  uint8_t stream_requesters_{0};
  uint8_t single_requesters_{0};

  // V4L2 state.
  //
  // A direct source (USB-UVC, or an explicit /dev/videoN already producing
  // JPEG/MJPEG) only uses capture_fd_ + capture_buffers_.
  //
  // The hardware-JPEG source spans two devices: capture_fd_ is the MIPI-CSI/ISP
  // device producing RGB565 frames, jpeg_fd_ is the JPEG hardware encoder (an
  // M2M device) fed RGB565 on its OUTPUT queue and read as JPEG from its CAPTURE
  // queue (jpeg_out_buffer_).
  int capture_fd_{-1};
  int jpeg_fd_{-1};
  bool streaming_{false};
  uint32_t capture_width_{0};
  uint32_t capture_height_{0};
  static constexpr int MAX_BUFFERS = 3;
  struct MappedBuffer {
    void *start{nullptr};
    size_t length{0};
  };
  MappedBuffer capture_buffers_[MAX_BUFFERS];
  int num_capture_buffers_{0};
  MappedBuffer jpeg_out_buffer_;

  // FORK: capture task + frame hand-off to loop().
  TaskHandle_t capture_task_{nullptr};
  SemaphoreHandle_t frame_mutex_{nullptr};
  uint8_t *pending_jpeg_{nullptr};  // finished PSRAM copy, waiting for loop()
  size_t pending_jpeg_len_{0};
  std::atomic<bool> capture_wanted_{false};
  int dbg_frames_{0};  // DQBUF diagnostics: log only the first few frames

  // FORK (black frames): warmup — the first N frames after the capture pipeline
  // starts are dropped (AE/IPA needs ~10 frames to converge, otherwise an
  // event-triggered snapshot comes out black). linger — the capture pipeline
  // stays alive for another 5 s after the last request, so a burst of events is
  // served by an already warm camera instead of restarting the pipeline.
  int warmup_left_{0};          // capture task only
  uint32_t idle_since_ms_{0};   // loop() only
  static constexpr int WARMUP_FRAMES = 10;
  static constexpr uint32_t LINGER_MS = 5000;

  // FORK: runtime settings (HA / web): -1 means unset.
  std::atomic<int> rt_exposure_{-1};  // V4L2_CID_EXPOSURE, 2-235 on the OV5647
  std::atomic<int> rt_vflip_{-1};    // V4L2_CID_VFLIP 0/1
  std::atomic<int> rt_hflip_{-1};    // V4L2_CID_HFLIP 0/1
  std::atomic<int> rt_quality_{-1};  // V4L2_CID_JPEG_COMPRESSION_QUALITY 1-63
  std::atomic<bool> ctrls_dirty_{false};
};

}  // namespace esphome::esp_video_camera

#endif  // USE_ESP_IDF
