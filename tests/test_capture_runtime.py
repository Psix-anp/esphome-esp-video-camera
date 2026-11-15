"""Exercise actual camera functions with controlled V4L2/PPA adapters."""

from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components/esp_video_camera"


def block(text, marker):
    start = text.index(marker)
    opening = text.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def method(text, name):
    marker = "ESPVideoCamera::" + name + "("
    start = text.rfind("\n", 0, text.index(marker)) + 1
    return block(text, text[start : text.index(marker) + len(marker)])


@pytest.fixture(scope="module", params=[False, True], ids=["normal", "video-debug"])
def camera_binary(tmp_path_factory, request):
    folder = tmp_path_factory.mktemp("camera-owner")
    source = (COMPONENT / "esp_video_camera.cpp").read_text()
    header = (COMPONENT / "esp_video_camera.h").read_text()
    names = (
        "configure_capture_framerate_",
        "loop_jpeg_pipeline_",
        "loop_direct_capture_",
        "queue_frame_",
        "has_consumers_",
        "deliver_raw_frame_",
        "deliver_jpeg_frame_",
        "stop_raw_frame_consumer",
        "stop_jpeg_frame_consumer",
    )
    methods = [method(source, name) for name in names]
    declarations = "\n".join(
        body[: body.index("{")].replace("ESPVideoCamera::", "").strip() + ";"
        for body in methods
    )
    state = header[
        header.index("  std::atomic<float> max_framerate_") : header.index(
            "  // Consumers"
        )
    ]
    # Permit the same production-method oracle on the pre-admission baseline.
    if "jpeg_admission_active_" not in state:
        state += "\n bool jpeg_admission_active_=false;\n"
    enumeration = (
        block(header, "enum class CaptureRateMode") + ";"
        if "enum class CaptureRateMode" in header
        else ""
    )
    debug_start = header.index(
        "#ifdef USE_ESPHOME_VOIP_STACK_VIDEO_DEBUG",
        header.index("jpeg_frame_consumer_active_"),
    )
    debug = header[debug_start : header.index("#endif", debug_start) + len("#endif")]
    types = "\n".join(
        block(header, marker) + ";"
        for marker in (
            "enum class RawVideoPixelFormat",
            "struct RawVideoFrame",
            "class RawVideoFrameConsumer",
            "struct JpegFrame",
            "class JpegFrameConsumer",
        )
    )
    settings = (
        block(header, "void set_max_framerate(")
        + "\n"
        + block(header, "void set_runtime_max_fps(")
    )
    configure = (
        "cam.capture_rate_mode_ = cam.configure_capture_framerate_();"
        if "CaptureRateMode" in enumeration
        else "cam.hardware_framerate_active_.store(cam.configure_capture_framerate_());"
    )
    code = (
        r"""
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <linux/videodev2.h>
#if __has_include("frame_cadence.h")
#include "frame_cadence.h"
#endif
#define ESP_LOGI(...)
#define ESP_LOGW(...)
#define ESP_LOGE(...)
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT 2
#define portMAX_DELAY 0
constexpr int ESP_OK=0,ESP_FAIL=-1;
using esp_err_t=int;
using SemaphoreHandle_t=void*;
const char*esp_err_to_name(int){return "controlled-error";}
uint64_t clock_us=0;
uint32_t micros(){return clock_us;}
uint32_t millis(){return clock_us/1000;}
int64_t esp_timer_get_time(){return clock_us;}
void xSemaphoreTake(void*,int){}void xSemaphoreGive(void*){}
unsigned image_allocations=0;
void*heap_caps_malloc(size_t n,int){++image_allocations;return std::malloc(n);}
void heap_caps_free(void*p){std::free(p);}
uint8_t sensor[1280000]{},rotated[1280000]{},encoded[16]{1,2,3};
struct Device {
 unsigned reported_fps=50,last_set_fps=0,fps_cap=0,raw_qbuf=0,ppa=0,jpeg=0,ppa_us=60000,raw_bytes=1280000;
 bool get_fail=false,set_fail=false,confirm_mismatch=false,raw_owned=false,ppa_reading=false,jpeg_raw_owned=false,fail_raw_qbuf=false;
 std::function<void()>on_ppa_done;
 bool ppa_fail=false;unsigned fail_ioctl=0;
 std::vector<std::string>events;
} dev;
constexpr int PPA_SRM_COLOR_MODE_RGB565=1,PPA_TRANS_MODE_BLOCKING=1;
struct PpaSide {void*buffer=nullptr;uint32_t buffer_size=0,pic_w=0,pic_h=0,block_w=0,block_h=0;int srm_cm=0;};
struct ppa_srm_oper_config_t {PpaSide in,out;int rotation_angle=0,mode=0;float scale_x=1,scale_y=1;bool rgb_swap=false,byte_swap=false;};
int ppa_rotation_for_clockwise(unsigned){return 3;}
int ppa_do_scale_rotate_mirror(void*,ppa_srm_oper_config_t*){
 assert(dev.raw_owned);dev.ppa_reading=true;dev.events.push_back("ppa");++dev.ppa;clock_us+=dev.ppa_us;dev.ppa_reading=false;
 if(dev.on_ppa_done)dev.on_ppa_done();
 return dev.ppa_fail?ESP_FAIL:ESP_OK;
}
int fake_ioctl(int fd,unsigned long command,void*arg){
 if(command==VIDIOC_G_PARM){if(dev.get_fail){errno=ENOTSUP;return -1;}auto*p=static_cast<v4l2_streamparm*>(arg);p->parm.capture.capability=V4L2_CAP_TIMEPERFRAME;p->parm.capture.timeperframe={1,dev.reported_fps};return 0;}
 if(command==VIDIOC_S_PARM){dev.last_set_fps=static_cast<v4l2_streamparm*>(arg)->parm.capture.timeperframe.denominator;if(dev.set_fail){errno=ENOTSUP;return -1;}if(!dev.confirm_mismatch)dev.reported_fps=dev.fps_cap?std::min(dev.last_set_fps,dev.fps_cap):dev.last_set_fps;return 0;}
 auto*buffer=static_cast<v4l2_buffer*>(arg);
 if(fd==1){
  if(command==VIDIOC_DQBUF){assert(!dev.raw_owned);dev.raw_owned=true;buffer->index=0;buffer->bytesused=dev.raw_bytes;dev.events.push_back("raw_dq");return 0;}
  assert(command==VIDIOC_QBUF);assert(dev.raw_owned&&!dev.ppa_reading&&!dev.jpeg_raw_owned);++dev.raw_qbuf;dev.events.push_back("raw_q");if(dev.fail_raw_qbuf){errno=EIO;return -1;}dev.raw_owned=false;return 0;
 }
 assert(fd==2);
 const unsigned step=(command==VIDIOC_QBUF?(buffer->type==V4L2_BUF_TYPE_VIDEO_OUTPUT?1:2):(buffer->type==V4L2_BUF_TYPE_VIDEO_CAPTURE?3:4));
 if(dev.fail_ioctl==step){errno=EIO;return -1;}
 if(step==1){dev.jpeg_raw_owned=buffer->m.userptr==reinterpret_cast<unsigned long>(sensor);dev.events.push_back("jpeg_own");}
 if(step==3){++dev.jpeg;clock_us+=12000;buffer->bytesused=3;dev.events.push_back("jpeg_encode");}
 if(step==4){dev.jpeg_raw_owned=false;dev.events.push_back("jpeg_release");}
 return 0;
}
#define ioctl fake_ioctl
namespace esphome::esp_video_camera {
"""
        + types
        + r"""
struct RawCounter:RawVideoFrameConsumer {unsigned count=0;void consume_raw_video_frame(const RawVideoFrame&)override{++count;}};
struct JpegCounter:JpegFrameConsumer {unsigned count=0;void consume_jpeg_frame(const JpegFrame&)override{++count;}};
class ESPVideoCamera {
 public:
"""
        + enumeration
        + "\n"
        + state
        + debug
        + r"""
 int capture_fd_=1,jpeg_fd_=2;
 unsigned num_capture_buffers_=2;
 struct MappedBuffer{void*start;size_t length;};
 MappedBuffer capture_buffers_[3]{{sensor,sizeof(sensor)},{sensor,sizeof(sensor)},{sensor,sizeof(sensor)}};
 MappedBuffer jpeg_out_buffer_{encoded,sizeof(encoded)};
 bool is_hw_jpeg_=true,is_raw_csi_=false,ppa_transform_required_=true;
 unsigned startup_frames_remaining_=0,raw_warmup_until_ms_=0;
 unsigned capture_width_=800,capture_height_=800,capture_stride_bytes_=1600;
 size_t capture_frame_bytes_=sizeof(sensor),rotated_rgb565_alloc_size_=sizeof(rotated),rotated_rgb565_bytes_=sizeof(rotated);
 unsigned jpeg_width_=800,jpeg_height_=800,rotation_degrees_=270;
 RawVideoPixelFormat capture_pixel_format_=RawVideoPixelFormat::RGB565_LE;
 float ppa_scale_x_=1,ppa_scale_y_=1;
 void*ppa_srm_=reinterpret_cast<void*>(1);uint8_t*rotated_rgb565_=rotated;
 std::atomic<bool>capture_faulted_{false},capture_wanted_{true};
 std::atomic<uint8_t>single_requesters_{0},stream_requesters_{0};
 std::atomic<RawVideoFrameConsumer*>raw_frame_consumer_{nullptr};
 std::atomic<JpegFrameConsumer*>jpeg_frame_consumer_{nullptr};
 std::atomic<bool>raw_frame_consumer_active_{false},jpeg_frame_consumer_active_{false};
 uint8_t*pending_jpeg_=nullptr;size_t pending_jpeg_len_=0;uint8_t pending_requesters_=0;
 unsigned last_alloc_warning_ms_=0;std::atomic<unsigned>capture_retry_attempts_{0};
 void*frame_mutex_=nullptr;unsigned loop_wakes=0;
 void enable_loop_soon_any_context(){++loop_wakes;}
 void update_capture_state_(){}
 ~ESPVideoCamera(){heap_caps_free(pending_jpeg_);}
"""
        + settings
        + "\n"
        + declarations
        + "\n};\n"
        + "\n".join(methods)
        + r"""
}
using namespace esphome::esp_video_camera;
int main(int argc,char**argv){
 assert(argc==2);const std::string scenario=argv[1];ESPVideoCamera cam;RawCounter raw;JpegCounter jpeg;
 cam.jpeg_frame_consumer_=&jpeg;cam.jpeg_frame_consumer_active_=true;
"""
        + configure
        + r"""
 if(scenario=="native-rate"){assert(dev.reported_fps==50&&"HW JPEG must retain the native source clock");return 0;}
 if(scenario=="raw-rate"){cam.is_hw_jpeg_=false;cam.is_raw_csi_=true;dev.reported_fps=50;"""
        + configure
        + r"""assert(dev.reported_fps==10);return 0;}
 if(scenario=="clamped-mixed"||scenario=="clamped-raw-only"){
  dev.reported_fps=25;dev.fps_cap=25;dev.ppa_us=0;
"""
        + configure
        + r"""
  cam.raw_frame_consumer_=&raw;cam.raw_frame_consumer_active_=true;
  const bool mixed=scenario=="clamped-mixed";cam.jpeg_frame_consumer_active_=mixed;cam.stream_requesters_=mixed?1:0;
  const unsigned frames=dev.reported_fps+1;
  for(unsigned i=0;i<frames;++i){clock_us=static_cast<uint64_t>(i)*1000000/dev.reported_fps;cam.loop_jpeg_pipeline_();}
  assert(raw.count<=11&&"HW JPEG max_framerate must cap RAW even if the driver clamps native FPS");
  assert(raw.count==11&&dev.raw_qbuf==frames&&!dev.raw_owned);
  assert(jpeg.count==(mixed?11U:0U)&&dev.ppa==(mixed?11U:0U)&&dev.jpeg==(mixed?11U:0U));
  assert(image_allocations==(mixed?11U:0U));return 0;
 }
 if(scenario=="restore-native"){
  dev.reported_fps=10;"""
        + configure
        + r"""
  assert(dev.last_set_fps==50&&dev.reported_fps==50&&cam.jpeg_admission_active_);return 0;
 }
 if(scenario=="unknown-rate"||scenario=="unknown-rate-raw-only"||
    scenario=="unsupported-set"||scenario=="unsupported-set-raw-only"||
    scenario=="unconfirmed-set"||scenario=="unconfirmed-set-raw-only"){
  const bool unknown=scenario.rfind("unknown-rate",0)==0;
  dev.reported_fps=unknown?50:25;dev.get_fail=unknown;
  if(unknown)cam.native_capture_fps_=0;
  dev.set_fail=scenario.rfind("unsupported-set",0)==0;
  dev.confirm_mismatch=scenario.rfind("unconfirmed-set",0)==0;dev.ppa_us=0;
"""
        + configure
        + r"""
  assert(cam.jpeg_admission_active_&&!cam.hardware_framerate_active_);
  if(!unknown)assert(dev.last_set_fps==50);
  cam.raw_frame_consumer_=&raw;cam.raw_frame_consumer_active_=true;
  const bool mixed=scenario.find("raw-only")==std::string::npos;
  cam.jpeg_frame_consumer_active_=mixed;cam.stream_requesters_=mixed?1:0;
  const unsigned frames=dev.reported_fps+1;
  for(unsigned i=0;i<frames;++i){clock_us=static_cast<uint64_t>(i)*1000000/dev.reported_fps;cam.loop_jpeg_pipeline_();}
  assert(raw.count==11&&dev.raw_qbuf==frames&&!dev.raw_owned);
  assert(jpeg.count==(mixed?11U:0U)&&dev.ppa==(mixed?11U:0U)&&dev.jpeg==(mixed?11U:0U));
  assert(image_allocations==(mixed?11U:0U));return 0;
 }
 if(scenario=="mixed") {cam.raw_frame_consumer_=&raw;cam.raw_frame_consumer_active_=true;}
 if(scenario=="api"||scenario=="runtime-api") {cam.jpeg_frame_consumer_active_=false;cam.stream_requesters_=1;}
 if(scenario=="runtime-api")cam.set_runtime_max_fps(10);
 if(scenario=="shared") {cam.raw_frame_consumer_=&raw;cam.raw_frame_consumer_active_=true;cam.stream_requesters_=1;cam.single_requesters_=2;}
 if(scenario=="single") {cam.jpeg_frame_consumer_active_=false;cam.single_requesters_=1;}
 if(scenario=="inactive") {cam.jpeg_frame_consumer_active_=false;cam.loop_jpeg_pipeline_();assert(dev.raw_qbuf==1&&dev.ppa==0&&dev.jpeg==0);return 0;}
 if(scenario=="raw-only") {cam.jpeg_frame_consumer_active_=false;cam.raw_frame_consumer_=&raw;cam.raw_frame_consumer_active_=true;}
 if(scenario=="startup") {cam.startup_frames_remaining_=1;cam.loop_jpeg_pipeline_();assert(dev.raw_qbuf==1&&dev.ppa==0);return 0;}
 if(scenario=="ppa-error") {dev.ppa_fail=true;cam.loop_jpeg_pipeline_();assert(dev.raw_qbuf==1&&dev.ppa==1&&dev.jpeg==0);return 0;}
 if(scenario=="empty-frame"){dev.raw_bytes=0;cam.loop_jpeg_pipeline_();assert(dev.raw_qbuf==1&&dev.ppa==0&&dev.jpeg==0&&jpeg.count==0);return 0;}
 if(scenario=="direct"){
  cam.is_hw_jpeg_=false;cam.jpeg_admission_active_=false;cam.hardware_framerate_active_=false;cam.stream_requesters_=1;dev.raw_bytes=3;
  for(unsigned t:{1000U,51000U,101000U}){clock_us=t;cam.loop_direct_capture_();}
  assert(jpeg.count==3&&image_allocations==2&&dev.raw_qbuf==3&&dev.ppa==0&&dev.jpeg==0);return 0;
 }
 if(scenario=="dedicated-raw"){
  cam.is_hw_jpeg_=false;cam.is_raw_csi_=true;cam.jpeg_admission_active_=false;cam.jpeg_frame_consumer_active_=false;cam.raw_frame_consumer_=&raw;cam.raw_frame_consumer_active_=true;
  for(unsigned t:{1000U,81000U}){clock_us=t;cam.loop_jpeg_pipeline_();}
  assert(raw.count==2&&dev.raw_qbuf==2&&dev.ppa==0&&dev.jpeg==0);return 0;
 }
 if(scenario=="stop-during-ppa"){
  dev.on_ppa_done=[&]{cam.stop_jpeg_frame_consumer(&jpeg);};cam.loop_jpeg_pipeline_();
  assert(jpeg.count==0&&dev.raw_qbuf==1&&!dev.raw_owned);clock_us=80000;cam.loop_jpeg_pipeline_();
  assert(dev.ppa==1&&dev.jpeg==1&&dev.raw_qbuf==2);return 0;
 }
 if(scenario.rfind("encoder-",0)==0){
  const bool rotation=scenario.rfind("encoder-rot-",0)==0;cam.ppa_transform_required_=rotation;
  dev.fail_ioctl=static_cast<unsigned>(scenario.back()-'0');cam.loop_jpeg_pipeline_();
  assert(dev.raw_qbuf==(rotation||dev.fail_ioctl==1?1U:0U));
  assert(cam.capture_faulted_==(dev.fail_ioctl!=1));assert(jpeg.count==0);return 0;
 }
 clock_us=0;cam.loop_jpeg_pipeline_();const auto first_ppa=dev.ppa,first_jpeg=dev.jpeg;
 if(scenario=="single"){assert(cam.single_requesters_==0&&image_allocations==1);clock_us=80000;cam.loop_jpeg_pipeline_();assert(dev.ppa==first_ppa&&dev.jpeg==first_jpeg);return 0;}
 if(scenario=="stop"){cam.stop_jpeg_frame_consumer(&jpeg);clock_us=80000;cam.loop_jpeg_pipeline_();assert(dev.ppa==first_ppa&&jpeg.count==1);return 0;}
 if(scenario=="cadence-qbuf-error")dev.fail_raw_qbuf=true;
 clock_us=80000;cam.loop_jpeg_pipeline_();
 if(scenario=="cadence-qbuf-error"){
  assert(dev.raw_qbuf==2&&cam.capture_faulted_&&!cam.capture_wanted_&&dev.ppa==first_ppa&&dev.jpeg==first_jpeg);return 0;
 }
 assert(dev.ppa==first_ppa&&dev.jpeg==first_jpeg&&"cadence rejection must precede PPA and JPEG");
 assert(dev.raw_qbuf==2&&!dev.raw_owned);
 if(scenario=="raw-only")assert(raw.count==1);
 if(scenario=="mixed"||scenario=="shared")assert(raw.count==1&&jpeg.count==1);
 if(scenario=="shared")assert(cam.single_requesters_==0&&cam.pending_requesters_==3&&image_allocations==1);
 if(scenario=="api"||scenario=="runtime-api")assert(image_allocations==1&&cam.pending_jpeg_!=nullptr);
 if(scenario!="api"&&scenario!="runtime-api"&&scenario!="shared")assert(image_allocations==0);
 if(scenario=="runtime-api")dev.ppa_us=56000;
 clock_us=100000;cam.loop_jpeg_pipeline_();assert(dev.raw_qbuf==3&&!dev.raw_owned);
 if(scenario=="raw-only")assert(raw.count==2&&dev.ppa==0&&dev.jpeg==0);
 else assert(dev.ppa==2&&dev.jpeg==2);
 if(scenario=="mixed"||scenario=="shared")assert(raw.count==2&&jpeg.count==2);
 if(scenario=="api"||scenario=="runtime-api"||scenario=="shared")assert(image_allocations==2&&cam.pending_jpeg_!=nullptr);
 if(scenario=="shared")assert(cam.single_requesters_==0&&cam.pending_requesters_==3);
 if(scenario!="api"&&scenario!="runtime-api"&&scenario!="shared")assert(image_allocations==0);
}
"""
    )
    cpp = folder / "camera.cpp"
    cpp.write_text(code)
    binary = folder / "camera"
    flags = ["-DUSE_ESPHOME_VOIP_STACK_VIDEO_DEBUG"] if request.param else []
    compiled = subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-g",
            "-fsanitize=address,undefined",
            "-fno-pie",
            "-no-pie",
            *flags,
            "-I",
            str(COMPONENT),
            str(cpp),
            "-o",
            str(binary),
        ],
        capture_output=True,
        text=True,
    )
    assert compiled.returncode == 0, compiled.stderr
    return binary


@pytest.mark.parametrize(
    "scenario",
    [
        "native-rate",
        "raw-rate",
        "unknown-rate",
        "frames",
        "mixed",
        "api",
        "single",
        "inactive",
        "raw-only",
        "startup",
        "ppa-error",
        "stop",
        "encoder-1",
        "encoder-2",
        "encoder-3",
        "encoder-4",
        "encoder-rot-1",
        "encoder-rot-2",
        "encoder-rot-3",
        "encoder-rot-4",
        "runtime-api",
        "shared",
        "direct",
        "dedicated-raw",
        "empty-frame",
        "stop-during-ppa",
        "cadence-qbuf-error",
        "restore-native",
        "unsupported-set",
        "unconfirmed-set",
        "clamped-mixed",
        "clamped-raw-only",
        "unknown-rate-raw-only",
        "unsupported-set-raw-only",
        "unconfirmed-set-raw-only",
    ],
)
def test_capture_admission_and_buffer_ownership(camera_binary, scenario):
    result = subprocess.run(
        [camera_binary, scenario], capture_output=True, text=True, timeout=8
    )
    assert result.returncode == 0, result.stdout + result.stderr
