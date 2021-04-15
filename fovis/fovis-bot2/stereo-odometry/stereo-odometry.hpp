#ifndef __fovis_bot2_stereo_odometry_hpp__
#define __fovis_bot2_stereo_odometry_hpp__

#include <iostream>
#include <fstream>

#include <signal.h>
#include <getopt.h>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <lcm/lcm.h>
#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_core.h>

#ifdef USE_LCMGL
#include <bot_lcmgl_client/lcmgl.h>
#endif

#include <fovis/fovis.hpp>

namespace fovis
{

class Visualization;

class StereoOdometry {
public:
  StereoOdometry();

  int initialize(int argc, char **argv);

  ~StereoOdometry();

  void go() {
    while((lcm_handle(_subscribe_lcm)==0) && !_shutdown_flag) {
      // nothing
    }
    if (_publish_tictoc) {
      publish_tictoc_stats();
    }

    fprintf(stderr, "Done.\n");
  }

private:
  StereoOdometry (const StereoOdometry& other);
  StereoOdometry& operator=(const StereoOdometry& other);

  static void sig_handler_aux(int signal, siginfo_t *s, void *data) {
    fprintf(stderr, "Shutting down\n");
    _shutdown_flag = 1;
  }

  static void image_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_image_t* msg,
                                void* user_data) {
    ((StereoOdometry *) user_data)->image_handler(msg);
  }

  static VisualOdometryOptions getDefaultOptions();

  int init_lcm();
  int init_calibration(const char * key_prefix);

  void image_handler(const bot_core_image_t *msg);
  void decode_image(const bot_core_image_t *msg);
  void publish_motion_estimation();
  void publish_tictoc_stats();

  void usage(const char *prog_name);
  int parse_command_line_options(int argc, char **argv);

  // data members
  static sig_atomic_t _shutdown_flag;

  lcm_t* _subscribe_lcm;
  lcm_t* _publish_lcm;
  lcm_t* _bot_param_lcm;

  std::string _odom_channel;
  std::string _pose_channel;
  std::string _frame_update_channel;
  std::string _stats_channel;
  std::string _tictoc_channel;

  BotParam* _bot_param;
  VisualOdometry* _odom;
  StereoDepth* _depth_producer;
  StereoCalibrationParameters _stereo_params;

  std::string _input_log_fname;
  std::string _output_log_fname;

  std::string _vo_parameters_fname;
  std::string _camera_parameters_fname;

  bool _publish_pose;
  bool _publish_odometry;
  bool _publish_frame_update;
  bool _publish_stats;
  bool _publish_tictoc;

  int64_t _utime_cur;
  int64_t _utime_prev;

  uint8_t* _image_left_buf;
  uint8_t* _image_right_buf;
  uint8_t* _images_buf;
  size_t _buf_size;

#ifdef USE_LCMGL
  bool _draw_lcmgl;
  Visualization* _visualization;
#endif

};

}

#endif /* end of include guard: __fovis_bot2_stereo_odometry_hpp__ */
