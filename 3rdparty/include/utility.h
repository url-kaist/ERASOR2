#ifndef UTILITY_H
#define UTILITY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <filesystem> // for directory reading
#include <string>
#include <dirent.h>
#include <algorithm>
#include <variant>
#include <iomanip>
#include <cstdlib>
#include <chrono>

const std::string green = "\033[32m";  // Green text
const std::string red = "\033[31m";    // Red text
const std::string yellow = "\033[33m"; // Yellow for status
const std::string cyan = "\033[36m";   // Cyan for input
const std::string reset = "\033[0m";   // Reset to default colors

enum LiDARType
{
  OUSTER = 0,
  VELODYNE = 1,
  LIVOX = 2,
  AEVA = 3
};

struct LivoxPointXYZI
{
  PCL_ADD_POINT4D;
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;
  uint32_t offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZI,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, reflectivity, reflectivity)(uint8_t, tag, tag)(uint8_t, line, line)(uint32_t, offset_time, offset_time))

struct AevaPointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  float reflectivity;
  float velocity;
  int32_t time_offset_ns;
  uint8_t line_index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(AevaPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, reflectivity, reflectivity)(float, velocity, velocity)(int32_t, time_offset_ns, time_offset_ns)(uint8_t, line_index, line_index))

struct OusterPointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ring;
  uint16_t ambient;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint16_t, ring, ring)(uint16_t, ambient, ambient))

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct Point3D
{
  float x;
  float y;
  float z;
};

std::string padZeros(int val, int num_digits);

bool isInFOV(double x, double y, double z); // Checks if a point is in the FOV of the sensor (Livox Accumulation)

float euclidean_distance(Point3D p1, Point3D p2);

#endif // UTILITY_H