#ifndef CAMERA_H_
#define CAMERA_H_

#define CHECK(cond, msg) do {if (!(cond)) {fprintf(stderr, "ERROR(%s:%d): %s\n", __FILE__, __LINE__, (msg)); exit(1);}} while (false);

#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_connection.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <opencv2/core/core.hpp>

class Camera {
 public:
  Camera(int width, int height) : width_(width), height_(height) {
    bcm_host_init();
    CHECK(mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera_) == MMAL_SUCCESS, "Failed to create a camera component.");
    MMAL_PORT_T *preview_port = camera_->output[0];
    MMAL_PORT_T *video_port = camera_->output[1];
    MMAL_PORT_T *still_port = camera_->output[2];

    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
      {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
      .max_stills_w = (uint32_t)width_,
      .max_stills_h = (uint32_t)height_,
      .stills_yuv422 = 0,
      .one_shot_stills = 1,
      .max_preview_video_w = 1280,
      .max_preview_video_h = 720,
      .num_preview_video_frames = 3,
      .stills_capture_circular_buffer_height = 0,
      .fast_preview_resume = 0,
      .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
    };
    CHECK(mmal_port_parameter_set(camera_->control, &cam_config.hdr) == MMAL_SUCCESS, "Failed to set the port parameter.");

    MMAL_ES_FORMAT_T *format;
    format = preview_port->format;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = VCOS_ALIGN_UP(1280, 32);
    format->es->video.height = VCOS_ALIGN_UP(720, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = 1280;
    format->es->video.crop.height = 720;
    format->es->video.frame_rate.num = 0;
    format->es->video.frame_rate.den = 1;
    CHECK(mmal_port_format_commit(preview_port) == MMAL_SUCCESS, "Failed to commit the preview port format.");

    mmal_format_full_copy(video_port->format, format);
    CHECK(mmal_port_format_commit(video_port) == MMAL_SUCCESS, "Failed to commit the video port format.");

    format = still_port->format;
    format->encoding = MMAL_ENCODING_BGR24;
    format->encoding_variant = 0;
    format->es->video.width = VCOS_ALIGN_UP(width_, 32);
    format->es->video.height = VCOS_ALIGN_UP(height_, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = width_;
    format->es->video.crop.height = height_;
    format->es->video.frame_rate.num = 0;
    format->es->video.frame_rate.den = 1;
    still_port->buffer_size = VCOS_ALIGN_UP(width_, 32) * VCOS_ALIGN_UP(height_, 16) * 3;
    still_port->buffer_num = 2;  // For MMAL_BUFFER_HEADER_FLAG_FRAME_END and MMAL_BUFFER_HEADER_FLAG_EOS.
    CHECK(mmal_port_format_commit(still_port) == MMAL_SUCCESS, "Failed to commit the still port");

    CHECK(mmal_component_enable(camera_) == MMAL_SUCCESS, "Failed to enable the camera component.");
    CHECK(mmal_port_parameter_set_boolean(still_port, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE) == MMAL_SUCCESS, "Failed to set zero copy.");
    CHECK(pool_ = mmal_port_pool_create(still_port, still_port->buffer_num, still_port->buffer_size), "Failed to create a pool.");

    CHECK(mmal_component_create("vc.null_sink", &preview_) == MMAL_SUCCESS, "Failed to create a null sink.");
    CHECK(mmal_component_enable(preview_) == MMAL_SUCCESS, "Failed to enable the preview.");
    CHECK(mmal_connection_create(&connection_, camera_->output[0], preview_->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT) == MMAL_SUCCESS, "Failed to create a connection.");
    CHECK(mmal_connection_enable(connection_) == MMAL_SUCCESS, "Failed to enable the connection.");

    still_port->userdata = (struct MMAL_PORT_USERDATA_T *)this;
    CHECK(mmal_port_enable(still_port, callback) == MMAL_SUCCESS, "Failed to enable the still port.");
  }

  ~Camera() {
    if (connection_->is_enabled) mmal_connection_disable(connection_);
    mmal_connection_destroy(connection_);
    mmal_pool_destroy(pool_);
    if (preview_->is_enabled) mmal_component_disable(preview_);
    mmal_component_destroy(preview_);
    if (camera_->is_enabled) mmal_component_disable(camera_);
    mmal_component_destroy(camera_);
  }

  void Configure(const std::string &config) {  // config="<shutter_speed_in_us> <analog_gain> <digital_gain>"
    int shutter_speed;
    float analog_gain;
    float digital_gain;
    CHECK(std::sscanf(config.c_str(), "%d %f %f", &shutter_speed, &analog_gain, &digital_gain) == 3, "Invalid configuration.");
    
    CHECK(mmal_port_parameter_set_uint32(camera_->control, MMAL_PARAMETER_SHUTTER_SPEED, shutter_speed) == MMAL_SUCCESS, "Failed to set the shutter speed.");
    MMAL_RATIONAL_T rational = {0, 65536};
    rational.num = (int32_t)(analog_gain * 65536);
    CHECK(mmal_port_parameter_set_rational(camera_->control, MMAL_PARAMETER_ANALOG_GAIN, rational) == MMAL_SUCCESS, "Failed to set the analog gain.");
    rational.num = (int32_t)(digital_gain * 65536);
    CHECK(mmal_port_parameter_set_rational(camera_->control, MMAL_PARAMETER_DIGITAL_GAIN, rational) == MMAL_SUCCESS, "Failed to set the digital gain.");
  }

  cv::Mat Capture() {
    CHECK(mmal_port_parameter_set_boolean(camera_->output[2], MMAL_PARAMETER_CAPTURE, MMAL_TRUE) == MMAL_SUCCESS, "Failed to set capture.");
    vcos_semaphore_create(&semaphore_, "camera", 0);
    const int queues = mmal_queue_length(pool_->queue);
    for (int i = 0; i < queues; i++) {
      MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool_->queue);
      CHECK(buffer, "Failed to get a buffer.");
      CHECK(mmal_port_send_buffer(camera_->output[2], buffer) == MMAL_SUCCESS, "Failed to send a buffer.");
    }
    CHECK(vcos_semaphore_wait(&semaphore_) == VCOS_SUCCESS, "Failed to wait a semaphore.");

    mmal_buffer_header_mem_lock(buffer_);
    const int width = buffer_->length / 3 / height_;
    CHECK((int)buffer_->length == width * height_ * 3, "Invalid buffer size.");
    const cv::Mat image(height_, width, CV_8UC3, buffer_->data);
    mmal_buffer_header_mem_unlock(buffer_);
    mmal_buffer_header_release(buffer_);
    return image(cv::Rect(0, 0, width_, height_));
  }

 private:
  static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    Camera *camera = (Camera *)port->userdata;
    if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
      camera->buffer_ = buffer;
      CHECK(mmal_port_parameter_set_boolean(port, MMAL_PARAMETER_CAPTURE, MMAL_FALSE) == MMAL_SUCCESS, "Failed to unset capture.");
      vcos_semaphore_post(&camera->semaphore_);
    } else {
      mmal_buffer_header_release(buffer);
    }
  }

  const int width_;
  const int height_;
  MMAL_COMPONENT_T *camera_;
  MMAL_COMPONENT_T *preview_;
  MMAL_POOL_T *pool_;
  MMAL_CONNECTION_T *connection_;
  VCOS_SEMAPHORE_T semaphore_;
  MMAL_BUFFER_HEADER_T *buffer_;
};

#undef CHECK

#endif
