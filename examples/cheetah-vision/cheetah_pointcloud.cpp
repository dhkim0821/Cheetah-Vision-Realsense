// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max


#include <iostream>
#include <thread> 
#include <type_traits>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include "cheetah_pointcloud.h"
#include <unistd.h>

int main(int argc, char * argv[]) try
{
  printf("start poincloud processing\n");
  rs2::pointcloud pc;
  rs2::points points;
  rs2::pipeline D435pipe;
  rs2::config D435cfg;
  D435cfg.enable_stream(RS2_STREAM_DEPTH, 640,480, RS2_FORMAT_Z16, 90);
  D435pipe.start(D435cfg);

  LocalizationHandle localizationObject;
  vision_lcm.subscribe("state_estimator", &LocalizationHandle::handlePose, &localizationObject);
  std::thread localization_thread(&handleLCM);

  int iter(0);
  ++iter;

  if(iter < 2){
    // World heightmap initialization
    for(int i(0); i<1000;++i){
      for(int j(0); j<1000; ++j){
        world_heightmap.map[i][j] = 0.;
      }
    }
    // Traversability initialization
    for(int i(0); i<100;++i){
      for(int j(0); j<100; ++j){
        traversability.map[i][j] = 0;
      }
    }
  }

  while (true) { 
    auto D435frames = D435pipe.wait_for_frames();
    auto depth = D435frames.get_depth_frame();

    points = pc.calculate(depth);
    _ProcessPointCloudData(points);

    if(iter%1000 == 1) printf("point cloud loop is run\n");
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

void _ProcessPointCloudData(const rs2::points & points){


  // filter
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
  SE3::SE3Multi(global_to_robot, robot_to_D435, global_to_D435);
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

  cv::Mat no_step_mat, jump_mat;
  cv::threshold(grad_max, no_step_mat, 0.07, 1, 0);
  cv::threshold(grad_max, jump_mat, 0.5, 1, 0);
  cv::Mat traversability_mat(100, 100, CV_32S);
  traversability_mat = no_step_mat + jump_mat;

  for(int i(0); i<100; ++i){
    for(int j(0); j<100; ++j){
      local_heightmap.map[i][j] = cv_local_heightmap.at<double>(i,j);
      traversability.map[i][j] = traversability_mat.at<double>(i,j);
    }
  }
  (local_heightmap).robot_loc[0] = global_to_robot.xyz[0];
  (local_heightmap).robot_loc[1] = global_to_robot.xyz[1];
  (local_heightmap).robot_loc[2] = global_to_robot.xyz[2];


  vision_lcm.publish("local_heightmap", &local_heightmap);
  vision_lcm.publish("traversability", &traversability);
  //vision_lcm.publish("cf_pointcloud", &wf_pointcloud);
}
