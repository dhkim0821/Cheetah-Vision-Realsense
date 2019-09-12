#ifndef CHEETAH_VISION
#define CHEETAH_VISION

#include <lcm/lcm-cpp.hpp>
#include "SE3.hpp"

#define WORLDMAP_SIZE 1000
float LOCAL_MAP_SIZE = 1.5; // in meters
int CELLS_PER_M = ceil((float) 100 / LOCAL_MAP_SIZE);
int WORLD_SIZE = 10;

SE3 robot_to_D435(0.28, 0.0, -0.01, 0, 0.49, 0);
SE3 T265_to_robot(0.0, 0.0, 0.07, M_PI, 0., -M_PI/2.);
SE3 global_to_T265_frame(0.0, 0.0, 0.17, M_PI/2., 0.0, -M_PI/2.);
SE3 T265_frame_to_T265;

double square(double a) {  return a * a; }

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

state_estimator_lcmt state_estimator_pose;
lcm::LCM vision_lcm("udpm://239.255.76.67:7667?ttl=255");

//void EulerToSE3(double x, double y, double z, double r, double p, double yaw, SE3 & pose) // yaw (Z), pitch (Y), roll (X)
//{
  //pose.xyz[0] = x;
  //pose.xyz[1] = y;
  //pose.xyz[2] = z;

  //pose.R[0][0] = cos(yaw)*cos(p);  
  //pose.R[0][1] = cos(yaw)*sin(p)*sin(r) - sin(yaw)*cos(r); 
  //pose.R[0][2] = cos(yaw)*sin(p)*cos(r) + sin(yaw)*sin(r); 

  //pose.R[1][0] = sin(yaw)*cos(p);  
  //pose.R[1][1] = sin(yaw)*sin(p)*sin(r) + cos(yaw)*cos(r); 
  //pose.R[1][2] = sin(yaw)*sin(p)*cos(r) - cos(yaw)*sin(r); 

  //pose.R[2][0] = -sin(p);
  //pose.R[2][1] = cos(p)*sin(r); 
  //pose.R[2][2] = cos(p)*cos(r);
//}

void SE3Multi(const SE3 & a, const SE3 & b, SE3 & out){
  double p[3];
  for(int i(0); i<3; ++i){
    p[i] = 0;
    for(int k(0); k<3; ++k){
      out.R[i][k] =0;
      p[i] += a.R[i][k] * b.xyz[k];
      for(int j(0); j<3; ++j){
        out.R[i][k] += (a.R[i][j]*b.R[j][k]);
      }
    }
    out.xyz[i] = a.xyz[i] + p[i];
  }
}

void rsPoseToSE3(const rs2::pose_frame & pose_frame, SE3 & pose){
  auto pose_data = pose_frame.get_pose_data();
  //pose.xyz[0] = -(double) pose_data.translation.y;  
  //pose.xyz[1] = -(double) pose_data.translation.x;  
  //pose.xyz[2] = -(double) pose_data.translation.z;  

  pose.xyz[0] = (double) pose_data.translation.x;  
  pose.xyz[1] = (double) pose_data.translation.y;  
  pose.xyz[2] = (double) pose_data.translation.z;  


  double e0 = (double) pose_data.rotation.w; 
  double e1 = (double) pose_data.rotation.x;
  double e2 = (double) pose_data.rotation.y;
  double e3 = (double) pose_data.rotation.z;
  
  //double e0 = (double) pose_data.rotation.w; 
  //double e1 = -(double) pose_data.rotation.y;
  //double e2 = -(double) pose_data.rotation.x;
  //double e3 = -(double) pose_data.rotation.z;


  double r,p,y;
  double as = std::min(-2. * (e1 * e3 - e0 * e2), .99999);
  y =std::atan2(2 * (e1 * e2 + e0 * e3),
                 square(e0) + square(e1) - square(e2) - square(e3));
  p = std::asin(as);
  r = std::atan2(2 * (e2 * e3 + e0 * e1),
                 square(e0) - square(e1) - square(e2) + square(e3));
 


  pose.R[0][0] = 1 - 2 * (e2 * e2 + e3 * e3);
  pose.R[0][1] = 2 * (e1 * e2 - e0 * e3);
  pose.R[0][2] = 2 * (e1 * e3 + e0 * e2);

  pose.R[1][0] = 2 * (e1 * e2 + e0 * e3);
  pose.R[1][1] = 1 - 2 * (e1 * e1 + e3 * e3);
  pose.R[1][2] = 2 * (e2 * e3 - e0 * e1);

  pose.R[2][0] = 2 * (e1 * e3 - e0 * e2);
  pose.R[2][1] = 2 * (e2 * e3 + e0 * e1);
  pose.R[2][2] = 1 - 2 * (e1 * e1 + e2 * e2);

  // transpose
  //pose.R[0][0] = 1 - 2 * (e2 * e2 + e3 * e3);
  //pose.R[1][0] = 2 * (e1 * e2 - e0 * e3);
  //pose.R[2][0] = 2 * (e1 * e3 + e0 * e2);

  //pose.R[0][1] = 2 * (e1 * e2 + e0 * e3);
  //pose.R[1][1] = 1 - 2 * (e1 * e1 + e3 * e3);
  //pose.R[2][1] = 2 * (e2 * e3 - e0 * e1);

  //pose.R[0][2] = 2 * (e1 * e3 - e0 * e2);
  //pose.R[1][2] = 2 * (e2 * e3 + e0 * e1);
  //pose.R[2][2] = 1 - 2 * (e1 * e1 + e2 * e2);
}

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


rotMat_t poseToRotationMatrix(xyzq_pose_t pose){
  double e0 = pose.wxyz_quaternion[0]; // w x y z
  double e1 = pose.wxyz_quaternion[1];
  double e2 = pose.wxyz_quaternion[2];
  double e3 = pose.wxyz_quaternion[3];
  rotMat_t rotMat;

  rotMat.R[0][0] = 1-2*(e2*e2 + e3*e3);
  rotMat.R[1][0] = 2*(e1*e2 - e0*e3);
  rotMat.R[2][0] = 2*(e1*e3 + e0*e2);

  rotMat.R[0][1] = 2*(e1*e2 + e0*e3);
  rotMat.R[1][1] = 1-2*(e1*e1 + e3*e3);
  rotMat.R[2][1] = 2*(e2*e3 - e0*e1);

  rotMat.R[0][2] = 2*(e1*e3 - e0*e2);
  rotMat.R[1][2] = 2*(e2*e3 + e0*e1);
  rotMat.R[2][2] = 1-2*(e1*e1 + e2*e2);

  return rotMat;
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



xyzq_pose_t stateEstimatorToXYZQPose(state_estimator_lcmt state_estimate){

  xyzq_pose_t xyzq_pose; 
  xyzq_pose.xyz[0] = (double) state_estimate.p[0];	
  xyzq_pose.xyz[1] = (double) state_estimate.p[1];	
  xyzq_pose.xyz[2] = (double) state_estimate.p[2];	

  //xyzq_pose.xyz[0] = 0.;
  //xyzq_pose.xyz[1] = 0.;
  //xyzq_pose.xyz[2] = 0.;

  xyzq_pose.wxyz_quaternion[0] = (double) state_estimate.quat[0];	
  xyzq_pose.wxyz_quaternion[1] = (double) state_estimate.quat[1];
  xyzq_pose.wxyz_quaternion[2] = (double) state_estimate.quat[2];
  xyzq_pose.wxyz_quaternion[3] = (double) state_estimate.quat[3];


  return xyzq_pose;
}

void handleLCM() {
  while (true)  { 
    vision_lcm.handle();
  };
}

#endif
