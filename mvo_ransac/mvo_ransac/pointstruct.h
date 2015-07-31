#ifndef POINTSTRUCT_H
#define POINTSTRUCT_H  
#include "matrix.h"
#include<vector>

using namespace std;

struct p_match
	{
	float u1p;
	float v1p;
	float  u1c;
	float  v1c;
	};
  struct parameters{
    double                      height;           // camera height above ground (meters)
    double                      pitch;            // camera pitch (rad, negative=pointing down)
    int32_t                     ransac_iters;     // number of RANSAC iterations
    double                      inlier_threshold; // fundamental matrix inlier threshold
    double                      motion_threshold; // directly return false on small motions
    parameters () {
      height           = 1.0;
      pitch            = 0.0;
      ransac_iters     = 2000;//RANSAC 
      inlier_threshold = 0.00001;//
      motion_threshold = 100.0;
    }
  };

#endif