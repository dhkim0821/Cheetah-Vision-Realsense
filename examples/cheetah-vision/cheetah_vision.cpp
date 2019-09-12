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
#include "cheetah_vision.h"



void _ProcessPointCloudData(const rs2::points & points, const rs2::pose_frame & pose_frame);

int main(int argc, char * argv[]) try
{
  printf("start vision processing\n");
  rs2::context ctx; // Create librealsense context for managing devices

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;
  rs2_pose pose;
  std::vector<rs2::pipeline> pipelines;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  for (auto&& dev : ctx.query_devices())  {
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    printf("%s\n", dev.get_info(RS2_CAMERA_INFO_NAME) );
    std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
    if( (name.compare("Intel RealSense D435")) == 0 ){
      printf("stream higer rate with %s\n", name.c_str() );
      cfg.enable_stream(RS2_STREAM_DEPTH, 640,480, RS2_FORMAT_Z16, 90);
    }
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
  }
  
  //LidarPoseHandler lidarHandlerObject; 
  StateEstimatorPoseHandler stateEstimatorHandlerObject;
  vision_lcm.subscribe("state_estimator", &StateEstimatorPoseHandler::handlePose, &stateEstimatorHandlerObject);
  std::thread lidar_sub_thread(&handleLCM);


  static int count(0);
  while (true)
  {
    ++count;
    // Wait for the next set of frames from the camera
    auto D435frames = pipelines[0].wait_for_frames();
    auto depth = D435frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    // Wait for the next set of frames from the camera
    auto T265frames = pipelines[1].wait_for_frames();
    // Get a frame from the pose stream
    auto f = T265frames.first_or_default(RS2_STREAM_POSE);
    
    // Cast the frame to pose_frame and get its data
    auto pose_frame = f.as<rs2::pose_frame>();

    _ProcessPointCloudData(points, pose_frame);
    if(count%100 ==1) {
      printf("%d th iter point cloud data is processed\n", count);
    }
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


void _ProcessPointCloudData(const rs2::points & points, const rs2::pose_frame & pose_frame){
  static heightmap_t local_heightmap;
  static worldmap world_heightmap;
  static traversability_map_t traversability;
  static int iter(0);

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
  ++iter;

  int erosion_size = 2;
  static cv::Mat erosion_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), 
      cv::Point( erosion_size, erosion_size ) );
  int dilation_size = 2;
  static cv::Mat dilation_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), 
      cv::Point( dilation_size, dilation_size ) );

  static xyzq_pose_t COM_to_D435_TF = poseFromRPY(0.28, 0.0, -0.01, 0., 0.49, 0.0);
  //static xyzq_pose_t T265_to_COM_TF = poseFromRPY(0.0, 0.0, 0.07, 0., 1.5707, 0.);
  static xyzq_pose_t T265_to_COM_TF = poseFromRPY(0.0, 0.0, 0.07, -M_PI/2., 0.,0. );

  static SE3 robot_to_D435, T265_to_robot, global_to_T265_frame, T265_frame_to_T265; 
  EulerToSE3(0.28, 0.0, -0.01, 0, 0.49, 0, robot_to_D435);
  EulerToSE3(0.0, 0.0, 0.07, M_PI, 0., -M_PI/2., T265_to_robot);
  EulerToSE3(0.0, 0.0, 0.17, M_PI/2., 0.0, -M_PI/2., global_to_T265_frame);
  rsPoseToSE3(pose_frame, T265_frame_to_T265);

  static SE3 global_to_T265, global_to_robot, global_to_D435;

  SE3Multi(global_to_T265_frame, T265_frame_to_T265, global_to_T265);
  SE3Multi(global_to_T265, T265_to_robot, global_to_robot);
  //global_to_T265.print("G2T");
  //T265_to_robot.print("T2R");
  //global_to_robot.print("G2R");

  static SE3 initial_posture_correction; 
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
  static SE3 corrected_global_to_robot;
  SE3Multi(initial_posture_correction, global_to_robot, corrected_global_to_robot);
  SE3Multi(corrected_global_to_robot, robot_to_D435, global_to_D435);
  //corrected_global_to_robot.print("corrected G2R");

  //double rpy_corr[3];
  //corrected_global_to_robot.getRPY(rpy_corr);
  //printf("rpy_corr: %f, %f, %f\n", rpy_corr[0], rpy_corr[1], rpy_corr[2]);

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
  xyzq_pose_t state_estimator_xyzq = rsPoseToXYZQPose(pose_frame);
  //printf("%f, %f,%f\n", state_estimator_xyzq.xyz[0], state_estimator_xyzq.xyz[1], state_estimator_xyzq.xyz[2]);

  global_to_D435.pointcloudTransformation(cf_pointcloud, wf_pointcloud);

  wfPCtoHeightmap(&wf_pointcloud, &world_heightmap, 5000); //right

  extractLocalFromWorldHeightmap(&state_estimator_xyzq, &world_heightmap, &local_heightmap); // writes over local heightmap in place

  cv::Mat cv_local_heightmap(100, 100, CV_64F, local_heightmap.map);
  cv::dilate( cv_local_heightmap, cv_local_heightmap, dilation_element );	
  cv::erode( cv_local_heightmap, cv_local_heightmap, erosion_element );

  cv::Mat	grad_x, grad_y;
  cv::Sobel(cv_local_heightmap, grad_x, CV_64F, 1,0,3,1,0,cv::BORDER_DEFAULT);
  cv::Sobel(cv_local_heightmap, grad_y, CV_64F, 0,1,3,1,0,cv::BORDER_DEFAULT);
  cv::Mat abs_grad_x = abs(grad_x);
  cv::Mat abs_grad_y = abs(grad_y);
  cv::Mat grad_max = max(abs_grad_x, abs_grad_y);
  //cv::Mat grad_max = max(grad_x, grad_y);

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

  //printf("7\n");
  //if(iter%100 == 0){
    //for(int i(0); i<100; ++i){
      //for(int j(0); j<100; ++j){
        //printf("%f, ", traversability_mat.at<double>(i,j));
      //}
      //printf("\n");
    //}
      //printf("\n");
  //}

  //if(iter%100 == 0){
    //for(int i(0); i<100; ++i){
      //for(int j(0); j<100; ++j){
        //printf("%d, ", traversability.map[i][j]);
      //}
      //printf("\n");
    //}
      //printf("\n");
  //}


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

  (wf_pointcloud).position[0] = (state_estimator_xyzq).xyz[0];
  (wf_pointcloud).position[1] = (state_estimator_xyzq).xyz[1];
  (wf_pointcloud).position[2] = (state_estimator_xyzq).xyz[2];


  //printf("8\n");
  vision_lcm.publish("cf_pointcloud", &wf_pointcloud);
}
