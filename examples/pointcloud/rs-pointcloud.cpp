// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max


#include "../../../Cheetah-Software/lcm-types/cpp/rs_pointcloud_t.hpp" // ask if this makes sense
//#include "../../../Cheetah-Software/lcm-types/cpp/xyzq_pose_t.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/heightmap_t.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/traversability_map_t.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/state_estimator_lcmt.hpp"
#include <iostream>
#include <thread> 
#include <type_traits>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include "rs-pointcloud.h"



void _ProcessPointCloudData(const rs2::points & points);

int main(int argc, char * argv[]) try
{

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  //rs2::config cfg;
  //cfg.enable_stream(RS2_STREAM_DEPTH, 640,480, RS2_FORMAT_Z16, 90);
  // Start streaming with default recommended configuration
  //pipe.start(cfg);
  pipe.start();

  //LidarPoseHandler lidarHandlerObject; 
  StateEstimatorPoseHandler stateEstimatorHandlerObject;
  //vision_lcm.subscribe("LIDAR_POSE", &LidarPoseHandler::handlePose, &lidarHandlerObject);
  vision_lcm.subscribe("state_estimator", &StateEstimatorPoseHandler::handlePose, &stateEstimatorHandlerObject);
  //vision_lcm.subscribe("state_estimator_ctrl_pc", &StateEstimatorPoseHandler::handlePose, &stateEstimatorHandlerObject);
  std::thread lidar_sub_thread(&handleLCM);


  while (true)
  {
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    // Donghyun
    _ProcessPointCloudData(points);
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
  static heightmap_t local_heightmap;
  static worldmap world_heightmap;
  static int iter(0);

  if(iter == 0){
    for(int i(0); i<1000;++i){
      for(int j(0); j<1000; ++j){
        world_heightmap.map[i][j] = 0.;
      }
    }
  }
  ++iter;

  static traversability_map_t traversability;
  static int iter2(0);
  if(iter2 == 0){
    for(int i(0); i<100;++i){
      for(int j(0); j<100; ++j){
        traversability.map[i][j] = 0;
      }
    }		
  }
  ++iter2;

  int erosion_size = 2;
  static cv::Mat erosion_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), 
      cv::Point( erosion_size, erosion_size ) );
  int dilation_size = 2;
  static cv::Mat dilation_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), 
      cv::Point( dilation_size, dilation_size ) );

  //xyzq_pose_t lidar_to_camera_TF = poseFromRPY(0.28, 0.0, -0.124, 0.0, -0.417, 0.0);
  //static xyzq_pose_t COM_to_camera_TF = poseFromRPY(0.28, 0.0, -0.01, 0.0, -0.417, 0.0);
  static xyzq_pose_t COM_to_camera_TF = poseFromRPY(0.28, 0.0, -0.01, 0., 0.49, 0.0);


  static rs_pointcloud_t cf_pointcloud; // 921600
  static rs_pointcloud_t wf_pointcloud; 
  static rs_pointcloud_t rf_pointcloud;


  //std::cout<<points.size()<< std::endl;

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

  int num_skip = num_valid_points/5000;
  //int num_skip = 50;

  for (int i = 0; i < num_valid_points/num_skip; i++)
  {
    cf_pointcloud.pointlist[k][0] = vertices[valid_indices[i*num_skip]].z;
    cf_pointcloud.pointlist[k][1] = -vertices[valid_indices[i*num_skip]].x;
    cf_pointcloud.pointlist[k][2] = -vertices[valid_indices[i*num_skip]].y;
    ++k;
    if(k>5000){
      break;
    }
  }

  //printf("%dth iter) num of cf point: %d\n", iter, k);
  //printf("%dth iter) num of valid point: %d\n", iter, num_valid_points);
  //printf("%dth iter) num_skip: %d\n", iter, num_skip);

  coordinateTransformation_DH(COM_to_camera_TF, cf_pointcloud, rf_pointcloud);
  xyzq_pose_t state_estimator_xyzq = stateEstimatorToXYZQPose(state_estimator_pose);
  coordinateTransformation_DH(state_estimator_xyzq, rf_pointcloud, wf_pointcloud);


  wfPCtoHeightmap(&wf_pointcloud, &world_heightmap, 5000); //right

  extractLocalFromWorldHeightmap(&state_estimator_xyzq, &world_heightmap, &local_heightmap); // writes over local heightmap in place

  cv::Mat cv_local_heightmap(100, 100, CV_64F, local_heightmap.map);
  cv::dilate( cv_local_heightmap, cv_local_heightmap, dilation_element );	
  cv::erode( cv_local_heightmap, cv_local_heightmap, erosion_element );

  cv::Mat	grad_x, grad_y;
  cv::Sobel(cv_local_heightmap, grad_x, CV_64F, 1,0,3,1,0,cv::BORDER_DEFAULT);
  cv::Sobel(cv_local_heightmap, grad_y, CV_64F, 0,1,3,1,0,cv::BORDER_DEFAULT);
  cv::Mat grad_max = max(grad_x, grad_y);

  cv::Mat no_step_mat, jump_mat;
  cv::threshold(grad_max, no_step_mat, 0.015, 1, 0);
  cv::threshold(grad_max, jump_mat, 0.2, 1, 0);
  cv::Mat traversability_mat(100, 100, CV_32S, traversability.map);
  traversability_mat = no_step_mat + jump_mat;

  /*double jumpmin, jumpmax;
    cv::minMaxIdx(jump_mat, &jumpmin, &jumpmax);
    printf("jumpmin: %f, jumpmax: %f\n", jumpmin, jumpmax);

    double no_stepmin, no_stepmax;
    cv::minMaxIdx(no_step_mat, &no_stepmin, &no_stepmax);
    printf("no_stepmin: %f, no_stepmax: %f\n", no_stepmin, no_stepmax);

    double travmin, travmax;
    cv::minMaxIdx(traversability_mat, &travmin, &travmax);
    printf("travmin: %f, travmax: %f\n", travmin, travmax);

    std::cout<<"no step mat "<<no_step_mat.at<double>(50,0)<< "\n";	
    std::cout<<"jump mat "<<jump_mat.at<double>(50,0)<< "\n";
    std::cout<<"trav map "<<traversability.map[50][0]<< "\n";
    std::cout<<"trav mat "<<traversability_mat.at<double>(50,0)<< "\n";*/


  vision_lcm.publish("local_heightmap", &local_heightmap);
  vision_lcm.publish("traversability", &traversability);
  //vision_lcm.publish("cf_pointcloud", &rf_pointcloud);
  (wf_pointcloud).position[0] = (state_estimator_xyzq).xyz[0];
  (wf_pointcloud).position[1] = (state_estimator_xyzq).xyz[1];
  (wf_pointcloud).position[2] = (state_estimator_xyzq).xyz[2];


  vision_lcm.publish("cf_pointcloud", &wf_pointcloud);
  //vision_lcm.publish("cf_pointcloud", &cf_pointcloud);
}
