#ifndef SE3_CHEETAH
#define SE3_CHEETAH

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
    double xyz[3];
    double R[3][3];

  public:
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
