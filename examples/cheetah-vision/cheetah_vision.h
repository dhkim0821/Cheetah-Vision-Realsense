#ifndef CHEETAH_VISION
#define CHEETAH_VISION

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include "SE3.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/state_estimator_lcmt.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/localization_lcmt.hpp"

state_estimator_lcmt state_estimator_pose;
lcm::LCM vision_lcm("udpm://239.255.76.67:7667?ttl=255&recv_buf_size=2");

#define WORLDMAP_SIZE 1000
float LOCAL_MAP_SIZE = 1.5; // in meters
int CELLS_PER_M = ceil((float) 100 / LOCAL_MAP_SIZE);
int WORLD_SIZE = 10;

SE3 robot_to_D435(0.28, 0.0, -0.01, 0, 0.49, 0);
SE3 T265_to_robot(0.0, 0.0, 0.07, M_PI, 0., M_PI);
SE3 global_to_T265_frame(0.0, 0.0, 0.17, M_PI/2., 0.0, -M_PI/2.);

SE3 T265_frame_to_T265, global_to_T265, global_to_robot, global_to_D435;
SE3 corrected_global_to_robot;
SE3 initial_posture_correction; 
std::vector<rs2::pipeline> pipelines;

// Declare pointcloud object, for calculating pointclouds and texture mappings
rs2::pointcloud pc;
// We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points points;

std::thread pointcloud_thread;
std::thread localization_thread;

void pointcloud_loop();

void localization_loop(){
  static int iter(0);
  ++iter;
  if(iter%500 == 1){
    printf("localization loop is running: %d\n", iter);
  }
  // Wait for the next set of frames from the camera
  auto T265frames = pipelines[1].wait_for_frames();
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
  SE3::SE3Multi(corrected_global_to_robot, robot_to_D435, global_to_D435);
  
  //global_to_T265.print("G2T");
  //T265_to_robot.print("T2R");
  //global_to_robot.print("G2R");
  //corrected_global_to_robot.print("corrected G2R");
  localization_lcmt global_to_robot_lcm;
  corrected_global_to_robot.set_localization_lcmt(global_to_robot_lcm);
  vision_lcm.publish("global_to_robot", &global_to_robot_lcm);
}

void pointcloud_process_running(){
 while(true){ pointcloud_loop();
std::this_thread::yield();}
}

void localization_process_running(){
 while(true){ localization_loop(); 
std::this_thread::yield(); }
}
struct worldmap {
  double map[WORLDMAP_SIZE][WORLDMAP_SIZE];
};

struct rotMat_t {
  double R[3][3];
};

struct xyzq_pose_t{
  double xyz[3];
  double wxyz_quaternion[4];
};

xyzq_pose_t poseFromRPY(double x, double y, double z, double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
{
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  xyzq_pose_t pose;
  pose.xyz[0] = x;
  pose.xyz[1] = y;
  pose.xyz[2] = z;

  pose.wxyz_quaternion[0] = cy * cp * cr + sy * sp * sr;
  pose.wxyz_quaternion[1] = cy * cp * sr - sy * sp * cr;
  pose.wxyz_quaternion[2] = sy * cp * sr + cy * sp * cr;
  pose.wxyz_quaternion[3] = sy * cp * cr - cy * sp * sr;

  return pose;
}


void extractLocalFromWorldHeightmap(double* xyz, worldmap* worldmap_ptr, heightmap_t* local_heightmap_ptr)
{
  //Copy out local heightmap
  // 		calculate index of center of local map in world map
  int pose_x_ind = xyz[0]*CELLS_PER_M + WORLD_SIZE*CELLS_PER_M/2 -1; 
  int pose_y_ind = xyz[1]*CELLS_PER_M + WORLD_SIZE*CELLS_PER_M/2 -1;

  int UPPER_LIM = floor(((double)CELLS_PER_M) * LOCAL_MAP_SIZE);
  for (int i = 0; i < UPPER_LIM; i++)
  {
    for (int j = 0; j < UPPER_LIM; j++)
    {		
      int world_map_x_ind = pose_x_ind+i-UPPER_LIM/2;
      int world_map_y_ind = pose_y_ind+j-UPPER_LIM/2;
      if (world_map_x_ind >= 0 && world_map_x_ind < WORLD_SIZE*CELLS_PER_M && 
          world_map_y_ind >= 0 && world_map_y_ind < WORLD_SIZE*CELLS_PER_M)
      {
        (*local_heightmap_ptr).map[i][j] = (*worldmap_ptr).map[world_map_x_ind][world_map_y_ind];
      }			
    }
  }
}

void wfPCtoHeightmap(rs_pointcloud_t* wf_pointcloud_ptr, worldmap* world_heightmap_ptr, int num_valid_points){
  // Move world frame point cloud into world heightmap
  for (int i = 0; i < num_valid_points; i++)
  {
    int point_x_ind = (*wf_pointcloud_ptr).pointlist[i][0]*CELLS_PER_M + WORLD_SIZE*CELLS_PER_M/2 -1;
    int point_y_ind = (*wf_pointcloud_ptr).pointlist[i][1]*CELLS_PER_M + WORLD_SIZE*CELLS_PER_M/2 -1;
    if (point_x_ind >= 0 && point_x_ind < WORLD_SIZE*CELLS_PER_M && point_y_ind >= 0 && point_y_ind < WORLD_SIZE*CELLS_PER_M)
    {			
      (*world_heightmap_ptr).map[point_x_ind][point_y_ind] = (*wf_pointcloud_ptr).pointlist[i][2];
    }
  }
}

class StateEstimatorPoseHandler
{
  public:
    ~StateEstimatorPoseHandler() {}

    void handlePose(const lcm::ReceiveBuffer* rbuf, 
        const std::string& chan, 
        const state_estimator_lcmt* msg)
    {
      state_estimator_pose = *msg;
      //std::cout<<"receive lidar"<<std::endl;
    }
};

void handleLCM() {
  while (true)  { 
    vision_lcm.handle();
  };
}

#endif
