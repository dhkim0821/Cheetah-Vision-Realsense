#ifndef CHEETAH_VISION
#define CHEETAH_VISION

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include "SE3.hpp"

#include "../../../Cheetah-Software/lcm-types/cpp/localization_lcmt.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/rs_pointcloud_t.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/heightmap_t.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/traversability_map_t.hpp"

lcm::LCM vision_lcm("udpm://239.255.76.67:7667?ttl=255&recv_buf_size=10");

#define WORLDMAP_SIZE 1000

float LOCAL_MAP_SIZE = 1.5; // in meters
int CELLS_PER_M = ceil((float) 100 / LOCAL_MAP_SIZE);
int WORLD_SIZE = 10;

SE3 robot_to_D435(0.28, 0.0, -0.01, 0, 0.49, 0);
//SE3 robot_to_D435(0.30, 0.0, -0.01, 0, 0.49, 0);
SE3 global_to_robot, global_to_D435;

struct worldmap {  double map[WORLDMAP_SIZE][WORLDMAP_SIZE]; };

heightmap_t local_heightmap;
worldmap world_heightmap;
traversability_map_t traversability;


void _ProcessPointCloudData(const rs2::points & points);

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

class LocalizationHandle
{
  public:
    ~LocalizationHandle() {}

    void handlePose(const lcm::ReceiveBuffer* rbuf, 
        const std::string& chan, 
        const localization_lcmt* msg) {
      global_to_robot.get_localization_lcmt(msg);
    }
};

void handleLCM() {
  while (true)  { 
    vision_lcm.handle();
  };
}

#endif
