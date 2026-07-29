# esp_video 2.3.0 declares esp_h264 unconditionally for ESP32-P4 even though
# its sources and CMake dependency are gated by
# CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE. A local empty component lets a
# direct JPEG/MJPEG build satisfies that inactive manifest edge without
# downloading or compiling the H.264 implementation. Raw CSI integrations do
# not install this stub and can resolve the real managed component.
idf_component_register()
