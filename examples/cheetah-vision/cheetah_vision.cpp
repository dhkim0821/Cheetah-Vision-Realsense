// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max


#include "../../../Cheetah-Software/lcm-types/cpp/rs_pointcloud_t.hpp" // ask if this makes sense
#include "../../../Cheetah-Software/lcm-types/cpp/heightmap_t.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/traversability_map_t.hpp"
#include <iostream>
#include <thread> 
#include <type_traits>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include "cheetah_vision.h"
#include <unistd.h>


int main(int argc, char * argv[]) try
{
  printf("start vision processing\n");
  rs2::context ctx; // Create librealsense context for managing devices

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  for (auto&& dev : ctx.query_devices())  {
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    printf("%s\n", dev.get_info(RS2_CAMERA_INFO_NAME) );
    std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
    if( (name.compare("Intel RealSense D435")) == 0 ){
      //printf("stream higer rate with %s\n", name.c_str() );
      // Only when USB 3.0 is available
      // cfg.enable_stream(RS2_STREAM_DEPTH, 640,480, RS2_FORMAT_Z16, 90);
    }else{
      cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    }
    pipe.start(cfg);
    pipelines.push_back(pipe);
    //pipelines.emplace_back(pipe);
  }
  // StateEstimatorPoseHandler stateEstimatorHandlerObject;
  // vision_lcm.subscribe("state_estimator", &StateEstimatorPoseHandler::handlePose, 
  //   &stateEstimatorHandlerObject);
  // std::thread lidar_sub_thread(&handleLCM);

  pointcloud_thread = std::thread(&pointcloud_process_running);
  localization_thread = std::thread(&localization_process_running);

  while (true) { usleep(10000); }
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


void pointcloud_loop(){
  // Wait for the next set of frames from the camera
  auto D435frames = pipelines[0].wait_for_frames();
  auto depth = D435frames.get_depth_frame();

  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth);

  static heightmap_t local_heightmap;
  static worldmap world_heightmap;
  static traversability_map_t traversability;

  static int iter(0);
  ++iter;
  if(iter%100 == 1) printf("point cloud loop is run\n");

  if(iter < 2){
    for(int i(0); i<1000;++i){
      for(int j(0); j<1000; ++j){
        world_heightmap.map[i][j] = 0.;
      }
    }
  }

  if(iter < 2){
    for(int i(0); i<100;++i){
      for(int j(0); j<100; ++j){
        traversability.map[i][j] = 0;
      }
    }
  }

  int erosion_size = 2;
  static cv::Mat erosion_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), 
      cv::Point( erosion_size, erosion_size ) );
  int dilation_size = 2;
  static cv::Mat dilation_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), 
      cv::Point( dilation_size, dilation_size ) );

  static rs_pointcloud_t cf_pointcloud; // 921600
  static rs_pointcloud_t wf_pointcloud; 
  static rs_pointcloud_t TRS_pointcloud; // tracking realsense

  int num_points = points.size();
  auto vertices = points.get_vertices(); 

  // Move raw image into camera frame point cloud struct
  int k(0);

  int num_valid_points(0);
  std::vector<int> valid_indices; 

  for (int i = 0; i< num_points; i++)
  {
    if (vertices[i].z and vertices[i].z < 1.0)
    {
      num_valid_points++;
      valid_indices.push_back(i);
    }
  }

  int num_skip = floor(num_valid_points/5000);

  if(num_skip == 0){ num_skip = 1; }
  for (int i = 0; i < floor(num_valid_points/num_skip); i++)
  {
    cf_pointcloud.pointlist[k][0] = vertices[valid_indices[i*num_skip]].z;
    cf_pointcloud.pointlist[k][1] = -vertices[valid_indices[i*num_skip]].x;
    cf_pointcloud.pointlist[k][2] = -vertices[valid_indices[i*num_skip]].y;
    ++k;
    if(k>5000){
      break;
    }
  }
  global_to_D435.pointcloudTransformation(cf_pointcloud, wf_pointcloud);

  wfPCtoHeightmap(&wf_pointcloud, &world_heightmap, 5000); //right
  extractLocalFromWorldHeightmap(global_to_robot.xyz, &world_heightmap, &local_heightmap); // writes over local heightmap in place

  cv::Mat cv_local_heightmap(100, 100, CV_64F, local_heightmap.map);
  cv::dilate( cv_local_heightmap, cv_local_heightmap, dilation_element );	
  cv::erode( cv_local_heightmap, cv_local_heightmap, erosion_element );

  cv::Mat	grad_x, grad_y;
  cv::Sobel(cv_local_heightmap, grad_x, CV_64F, 1,0,3,1,0,cv::BORDER_DEFAULT);
  cv::Sobel(cv_local_heightmap, grad_y, CV_64F, 0,1,3,1,0,cv::BORDER_DEFAULT);
  cv::Mat abs_grad_x = abs(grad_x);
  cv::Mat abs_grad_y = abs(grad_y);
  cv::Mat grad_max = max(abs_grad_x, abs_grad_y);

  //printf("6\n");
  cv::Mat no_step_mat, jump_mat;
  cv::threshold(grad_max, no_step_mat, 0.07, 1, 0);
  cv::threshold(grad_max, jump_mat, 0.5, 1, 0);
  //cv::Mat traversability_mat(100, 100, CV_32S, traversability.map);
  cv::Mat traversability_mat(100, 100, CV_32S);
  traversability_mat = no_step_mat + jump_mat;

  for(int i(0); i<100; ++i){
    for(int j(0); j<100; ++j){
      local_heightmap.map[i][j] = cv_local_heightmap.at<double>(i,j);
      traversability.map[i][j] = traversability_mat.at<double>(i,j);
    }
  }
  (local_heightmap).robot_loc[0] = corrected_global_to_robot.xyz[0];
  (local_heightmap).robot_loc[1] = corrected_global_to_robot.xyz[1];
  (local_heightmap).robot_loc[2] = corrected_global_to_robot.xyz[2];


  vision_lcm.publish("local_heightmap", &local_heightmap);
  vision_lcm.publish("traversability", &traversability);
  //vision_lcm.publish("cf_pointcloud", &wf_pointcloud);
}
