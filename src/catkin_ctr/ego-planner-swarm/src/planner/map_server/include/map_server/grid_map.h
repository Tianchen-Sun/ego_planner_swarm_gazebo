/**
 * @file grid_map.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-01
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MPA_SERVER_GRID_MAP_H_
#define MPA_SERVER_GRID_MAP_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <vector>

class GridMap {
 private:
  // ros::Subscriber cld_sub_;
  ros::Publisher map_pub_;

  float _mp_resolution;
  float _mp_inflation;
  float _mp_resolution_inv;
  std::string _mp_frame_id;

  float _mp_size_x;
  float _mp_size_y;
  float _mp_size_z;

  int _mp_grid_size_x;
  int _mp_grid_size_y;
  int _mp_grid_size_z;

  int _inflate_size;
  int _inflate_size_z;

  bool is_map_built_;     // if map is already built

  Eigen::Vector3f _mp_origin_position;

  std::vector<bool> occupancy_buffer_;

  inline void boundIndex(Eigen::Vector3f pos);
  inline void posToIndex(const Eigen::Vector3f &pos, Eigen::Vector3i &id);
  inline int posToAddress(const Eigen::Vector3f &pos);
  inline int indexToAddress(const Eigen::Vector3i &pos);
  inline int indexToAddress(int &x, int &y, int &z);
  inline void indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3f &pos);
  inline Eigen::Vector3f indexToPos(int x, int y, int z);
  inline bool isIndexWithinBound(const Eigen::Vector3i idx);
  inline bool isPosWithinBound(const Eigen::Vector3f p);

 public:
  GridMap() {}
  ~GridMap() {}

  void initGridMap(ros::NodeHandle &nh);
  void initFromPointCloud(const sensor_msgs::PointCloud2ConstPtr &cld);
  void initBuffer(int grid_size_x, int grid_size_y, int grid_size_z);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cld);
  void publish();
  Eigen::Vector3i posToIndex(const Eigen::Vector3f &pos);
  Eigen::Vector3i posToIndex(const Eigen::Vector3d &pos);
  Eigen::Vector3d indexToPos(const Eigen::Vector3i &idx);
  bool isStraightLineCollision(const Eigen::Vector3f &start,
                               const Eigen::Vector3f &end);
  bool isPointCollision(const Eigen::Vector3f &p);
  bool isPointCollision(const Eigen::Vector3d &p);
  bool isPointCollision(const Eigen::Vector3i &i);
  bool isPointCollision(int& x, int& y, int& z);
  bool isMapBuilt();
  float getResolution();

  typedef std::shared_ptr<GridMap> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* MPA_SERVER_GRID_MAP_H_ */