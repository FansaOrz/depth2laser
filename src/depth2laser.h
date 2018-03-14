#pragma once
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

struct LaserConfig {
  float angle_min;
  float angle_max;
  int num_ranges;
  float range_min;
  float range_max;
  float laser_plane_thickness;
};

class Depth2Laser {
 public:

  Depth2Laser();

  //Set and get variables
  inline void setK(const Eigen::Matrix3f& K_) {_K = K_; _inv_K = K_.inverse();}
  inline Eigen::Matrix3f K() {return _K;}

  inline void setCamera2Laser(const Eigen::Isometry3f& camera2laser_transform_) {_camera2laser_transform = camera2laser_transform_;}
  inline Eigen::Isometry3f camera2Laser() {return _camera2laser_transform;}

  inline void setLaserConfig(const LaserConfig laser_config_) {_laser_config=laser_config_;}
  inline LaserConfig laserconfig() {return _laser_config;}

  inline void setImage(const cv::Mat depth_image_) {_depth_image=depth_image_;}
  inline cv::Mat depthimage() {return _depth_image;}

  //Computes de virtual scan from the depth image
  void compute();

  inline std::vector<float>& ranges() {return _ranges;}
  
 protected:

  //Camera variables
  Eigen::Matrix3f _K;//
  Eigen::Matrix3f _inv_K;//
  Eigen::Isometry3f _camera2laser_transform;//

  //Laser variables
  LaserConfig _laser_config;
  
  //Input
  cv::Mat _depth_image;

  //Output
  std::vector<float> _ranges;  
  
};
void Depth2Laser::compute()
{
  //if (skip_count<skip_frames) { skip_count++; return; }
  //skip_count=0;
  float angle_increment=(_laser_config.angle_max - _laser_config.angle_min)/_laser_config.num_ranges;
  float squared_max_norm = _laser_config.range_max*_laser_config.range_max;
  float squared_min_norm = _laser_config.range_min*_laser_config.range_min;
  float inverse_angle_increment = 1.0/angle_increment;
  int good_points = 0;

  _ranges.resize(_laser_config.num_ranges);

  for(size_t i=0; i<_ranges.size();i++)
    _ranges[i] = _laser_config.range_max + 0.1;

  for(int i=0; i < _depth_image.rows; i++)
  {
    const ushort* row_ptr = _depth_image.ptr<ushort>(i);
    for(int j=0; j< _depth_image.cols; j++)
    {
      ushort id = row_ptr[j];
      if(id != 0)
      {
        float d = 1e-3*id;
        Eigen::Vector3f image_point(j*d,i*d,d);
        Eigen::Vector3f camera_point = _inv_K*image_point;
        Eigen::Vector3f laser_point = _camera2laser_transform*camera_point;
        if(fabs(laser_point.z() < _laser_config.laser_plane_thickness))
        {
          float theta = atan2(laser_point.y(),laser_point.x());
          /*
          if(theta<scan.angle_min)
	          continue;
	        if(theta>scan.angle_max)
	          continue;
          */
          float range = laser_point.head<2>().squaredNorm();
          if(range < squared_min_norm)
            continue;
          if(range > squared_max_norm)
           continue;
          
          range=sqrt(range);
          int bin=(int)((theta - _laser_config.angle_min)*inverse_angle_increment);
          if(bin<0||bin>=_laser_config.num_ranges)
            continue;
          if(_ranges[bin]>range)
          {
            _ranges[bin] = range;
            good_points++;
          }
        }
      }
    }
  }
}