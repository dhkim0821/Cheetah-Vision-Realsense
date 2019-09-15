#ifndef SE3_CHEETAH
#define SE3_CHEETAH

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../Cheetah-Software/lcm-types/cpp/localization_lcmt.hpp"
#include "../../../Cheetah-Software/lcm-types/cpp/rs_pointcloud_t.hpp"

double square(double a) {  return a * a; }

class SE3{
  public:
    SE3(){}
    ~SE3(){}
    SE3(double x, double y, double z, double r, double p, double yaw) {
      xyz[0] = x;
      xyz[1] = y;
      xyz[2] = z;

      // ZYX implicit
      // yaw (Z), pitch (Y), roll (X)
      R[0][0] = cos(yaw)*cos(p);  
      R[0][1] = cos(yaw)*sin(p)*sin(r) - sin(yaw)*cos(r); 
      R[0][2] = cos(yaw)*sin(p)*cos(r) + sin(yaw)*sin(r); 

      R[1][0] = sin(yaw)*cos(p);  
      R[1][1] = sin(yaw)*sin(p)*sin(r) + cos(yaw)*cos(r); 
      R[1][2] = sin(yaw)*sin(p)*cos(r) - cos(yaw)*sin(r); 

      R[2][0] = -sin(p);
      R[2][1] = cos(p)*sin(r); 
      R[2][2] = cos(p)*cos(r);
    }
    double xyz[3] = {0., 0., 0.};
    double R[3][3] = 
    {{1., 0., 0.},
    {0., 1., 0.},
    {0., 0., 1.}};

  public:
    void set_localization_lcmt(localization_lcmt & lcm_t){
      double rpy[3];
      getRPY(rpy);
      for(int i(0); i<3; ++i){
        lcm_t.xyz[i] = xyz[i];
        lcm_t.rpy[i] = rpy[i];
      }
    }

    void get_localization_lcmt(const localization_lcmt * lcm_t){
      double r = lcm_t->rpy[0];
      double p = lcm_t->rpy[1];
      double yaw = lcm_t->rpy[2];

      R[0][0] = cos(yaw)*cos(p);  
      R[0][1] = cos(yaw)*sin(p)*sin(r) - sin(yaw)*cos(r); 
      R[0][2] = cos(yaw)*sin(p)*cos(r) + sin(yaw)*sin(r); 

      R[1][0] = sin(yaw)*cos(p);  
      R[1][1] = sin(yaw)*sin(p)*sin(r) + cos(yaw)*cos(r); 
      R[1][2] = sin(yaw)*sin(p)*cos(r) - cos(yaw)*sin(r); 

      R[2][0] = -sin(p);
      R[2][1] = cos(p)*sin(r); 
      R[2][2] = cos(p)*cos(r);
      
      for(int i(0); i<3; ++i){
        this->xyz[i] = lcm_t->xyz[i];
      }
    }


    void rsPoseToSE3(const rs2::pose_frame & pose_frame){
      auto pose_data = pose_frame.get_pose_data();

      xyz[0] = (double) pose_data.translation.x;  
      xyz[1] = (double) pose_data.translation.y;  
      xyz[2] = (double) pose_data.translation.z;  

      double e0 = (double) pose_data.rotation.w; 
      double e1 = (double) pose_data.rotation.x;
      double e2 = (double) pose_data.rotation.y;
      double e3 = (double) pose_data.rotation.z;

      //double r,p,y;
      //double as = std::min(-2. * (e1 * e3 - e0 * e2), .99999);
      //y =std::atan2(2 * (e1 * e2 + e0 * e3),
          //square(e0) + square(e1) - square(e2) - square(e3));
      //p = std::asin(as);
      //r = std::atan2(2 * (e2 * e3 + e0 * e1),
          //square(e0) - square(e1) - square(e2) + square(e3));

      R[0][0] = 1 - 2 * (e2 * e2 + e3 * e3);
      R[0][1] = 2 * (e1 * e2 - e0 * e3);
      R[0][2] = 2 * (e1 * e3 + e0 * e2);

      R[1][0] = 2 * (e1 * e2 + e0 * e3);
      R[1][1] = 1 - 2 * (e1 * e1 + e3 * e3);
      R[1][2] = 2 * (e2 * e3 - e0 * e1);

      R[2][0] = 2 * (e1 * e3 - e0 * e2);
      R[2][1] = 2 * (e2 * e3 + e0 * e1);
      R[2][2] = 1 - 2 * (e1 * e1 + e2 * e2);

      // transpose
      //R[0][0] = 1 - 2 * (e2 * e2 + e3 * e3);
      //R[1][0] = 2 * (e1 * e2 - e0 * e3);
      //R[2][0] = 2 * (e1 * e3 + e0 * e2);

      //R[0][1] = 2 * (e1 * e2 + e0 * e3);
      //R[1][1] = 1 - 2 * (e1 * e1 + e3 * e3);
      //R[2][1] = 2 * (e2 * e3 - e0 * e1);

      //R[0][2] = 2 * (e1 * e3 - e0 * e2);
      //R[1][2] = 2 * (e2 * e3 + e0 * e1);
      //R[2][2] = 1 - 2 * (e1 * e1 + e2 * e2);
    }
    static void SE3Multi(const SE3 & a, const SE3 & b, SE3 & out){
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


    void print(const std::string & name) const {
      printf("%s:\n", name.c_str());
      printf("%6.6f, %6.6f, %6.6f, %6.6f\n %6.6f, %6.6f, %6.6f, %6.6f\n %6.6f, %6.6f, %6.6f, %6.6f\n\n",
          R[0][0], R[0][1], R[0][2], xyz[0],
          R[1][0], R[1][1], R[1][2], xyz[1],
          R[2][0], R[2][1], R[2][2], xyz[2]);
    }
    void pointcloudTransformation(const rs_pointcloud_t & inputCloud, rs_pointcloud_t & outputCloud){
      int r1(3), c1(3), num_pts(5000);

      for (int pt_idx(0); pt_idx<num_pts; ++pt_idx){
        for (int i=0; i<r1; i++){
          outputCloud.pointlist[pt_idx][i] = xyz[i];
          for(int k=0; k<c1; k++){
            outputCloud.pointlist[pt_idx][i] += R[i][k]*inputCloud.pointlist[pt_idx][k];
          }
        }
      }
    }
    void getRPY(double* rpy){ // ZYX
      rpy[1] = std::asin(-R[2][0]);
      if(fabs(cos(rpy[1]) ) > 0.000001){
        rpy[0] = std::asin(R[2][1]/cos(rpy[1]));
        rpy[2] = std::atan2(R[1][0], R[0][0]);
      }else{
        //undefined
        rpy[0] = 0.;
        rpy[2] = 0.;
      }
    }
};



#endif
