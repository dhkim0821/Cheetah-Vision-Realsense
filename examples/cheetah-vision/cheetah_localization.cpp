// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <iostream>
#include <type_traits>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <unistd.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <lcm/lcm-cpp.hpp>
#include "SE3.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/localization_lcmt.hpp"

lcm::LCM vision_lcm("udpm://239.255.76.67:7667?ttl=255&recv_buf_size=3");

void handleLCM() {
  while (true)  { vision_lcm.handle(); };
}

int main(int argc, char * argv[]) try
{
  //SE3 T265_to_robot(0.0, 0.0, 0.07, M_PI, 0., -M_PI/2.); // parallel attachment of realsense
  SE3 T265_to_robot(0.0, 0.0, 0.07, M_PI, 0., M_PI);
  SE3 global_to_T265_frame(0.0, 0.0, 0.17, M_PI/2., 0.0, -M_PI/2.);

  SE3 T265_frame_to_T265, global_to_T265, global_to_robot, global_to_D435;
  SE3 corrected_global_to_robot;
  SE3 initial_posture_correction; 


  printf("Start Localization processing\n");
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  pipe.start(cfg);

  int iter(0);

  while (true) { 
    ++iter;
    if(iter%500 == 1){
      printf("localization loop is running: %d\n", iter);
    }
    // Wait for the next set of frames from the camera
    auto T265frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = T265frames.first_or_default(RS2_STREAM_POSE);

    // Cast the frame to pose_frame and get its data
    auto pose_frame = f.as<rs2::pose_frame>();

    T265_frame_to_T265.rsPoseToSE3(pose_frame);

    SE3::SE3Multi(global_to_T265_frame, T265_frame_to_T265, global_to_T265);
    SE3::SE3Multi(global_to_T265, T265_to_robot, global_to_robot);

    static double rpy[3];
    if(iter<2){
      global_to_robot.getRPY(rpy);
      initial_posture_correction.xyz[0] = 0.;
      initial_posture_correction.xyz[1] = 0.;
      initial_posture_correction.xyz[2] = 0.;

      initial_posture_correction.R[0][0] = cos(-rpy[2]);
      initial_posture_correction.R[0][1] = -sin(-rpy[2]);
      initial_posture_correction.R[0][2] = 0.;

      initial_posture_correction.R[1][0] = sin(-rpy[2]);
      initial_posture_correction.R[1][1] = cos(-rpy[2]);
      initial_posture_correction.R[1][2] = 0.;

      initial_posture_correction.R[2][0] = 0.;
      initial_posture_correction.R[2][1] = 0.;
      initial_posture_correction.R[2][2] = 1.;
    }
    //initial_posture_correction.print("correction");
    //printf("initial rpy: %f, %f, %f\n", rpy[0], rpy[1], rpy[2]);
    SE3::SE3Multi(initial_posture_correction, global_to_robot, corrected_global_to_robot);

    //global_to_T265.print("G2T");
    //T265_to_robot.print("T2R");
    //global_to_robot.print("G2R");
    //corrected_global_to_robot.print("corrected G2R");
    localization_lcmt global_to_robot_lcm;
    corrected_global_to_robot.set_localization_lcmt(global_to_robot_lcm);
    vision_lcm.publish("global_to_robot", &global_to_robot_lcm);
  }
  return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception & e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}


