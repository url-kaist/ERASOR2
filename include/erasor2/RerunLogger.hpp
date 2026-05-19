#ifndef ERASOR2_RERUN_LOGGER_HPP
#define ERASOR2_RERUN_LOGGER_HPP

// Thin wrapper around rerun::RecordingStream. Replaces every ROS publisher
// previously held in RosParamServer. Call sites turn from
//
//     CurrCloudPublisher.publish(erasor_utils::cloud2msg(cloud));
//
// into
//
//     erasor2::viz::logCloud("erasor2/curr_scan", cloud);
//
// Designed as free functions over a single global stream so we don't have to
// thread a logger pointer through every method that used to publish.

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace grid_map {
class GridMap;
}

namespace erasor2::viz {

// Initialize the global recording stream. Safe to call more than once;
// later calls become no-ops. Pass `spawn = true` to launch the rerun
// viewer subprocess; pass a non-empty `save_path` to record to a `.rrd`.
void init(const std::string& app_id    = "erasor2",
          bool spawn                   = true,
          const std::string& save_path = "");

// Disable all subsequent log calls. Mirrors the various viz_flag booleans.
void setEnabled(bool enabled);
bool isEnabled();

// Set the timeline cursor. Most call sites should use frame indices; rerun
// tags every following log call with this value so the viewer can scrub.
void setFrame(int64_t frame);

// --- log primitives ---------------------------------------------------

// Point cloud, intensity → grayscale by default. If `colors` non-empty it
// must be the same length as cloud and is used as-is (RGB triplets in [0,1]).
void logCloud(std::string_view path,
              const pcl::PointCloud<pcl::PointXYZI>& cloud,
              const std::vector<std::array<float, 3>>& colors = {});

// XYZRGB overload — uses the per-point r/g/b directly.
void logCloud(std::string_view path, const pcl::PointCloud<pcl::PointXYZRGB>& cloud);

// Path / pose history as a single line strip in world frame.
void logPath(std::string_view path, const std::vector<Eigen::Matrix4f>& poses);

// Pose as a Transform3D (position + rotation). Use this for `world/body`
// to drive parent-child transforms in the viewer.
void logPose(std::string_view path, const Eigen::Matrix4f& pose);

// 2D-grid occupancy / log-odds layer rendered as an image. `layer` is the
// grid_map layer name to render.
void logGridMapLayer(std::string_view path, const grid_map::GridMap& gm, const std::string& layer);

// Text label at a 3D position. Replaces the visualization_msgs::Marker
// TEXT_VIEW_FACING entries (object-score floats).
void logTextAt(std::string_view path,
               const Eigen::Vector3f& position,
               const std::string& text,
               const std::array<float, 3>& color = {1.0f, 1.0f, 1.0f});

// Cylinder (used for adaptive-range ring viz). Rerun has no native cylinder
// primitive; we emit a thin LineStrips3D circle at the given height.
void logCircle(std::string_view path,
               const Eigen::Vector3f& center,
               float radius,
               int num_segments = 64);

// --- ros::Publisher-shaped adapters -----------------------------------
// These mirror the surface of ros::Publisher.publish(...) so the legacy
// call sites can keep their syntax: `MyPublisher.publish(cloud)`. Each
// adapter carries a fixed rerun entity path and forwards to the free
// log* helpers above.

class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(std::string path) : path_(std::move(path)) {}

  // Point-cloud overloads (drop the legacy cloud2msg() round-trip).
  void publish(const pcl::PointCloud<pcl::PointXYZI>& cloud) const { logCloud(path_, cloud); }
  void publish(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) const { logCloud(path_, cloud); }

  // Pose overload.
  void publish(const Eigen::Matrix4f& pose) const { logPose(path_, pose); }

  // Path overload.
  void publish(const std::vector<Eigen::Matrix4f>& poses) const { logPath(path_, poses); }

  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

// Adapter for the few grid_map publishers — needs a layer name. Keeping
// it separate avoids the implicit conversion footgun where a 2D layer
// gets logged through the cloud overload.
class GridMapPublisher {
 public:
  GridMapPublisher() = default;
  explicit GridMapPublisher(std::string path) : path_(std::move(path)) {}
  void publish(const grid_map::GridMap& gm, const std::string& layer = "elevation") const {
    logGridMapLayer(path_, gm, layer);
  }

 private:
  std::string path_;
};

// Adapter for the marker-array text-score publishers. Original code
// pushed a list of (position, score) pairs as TEXT_VIEW_FACING markers;
// here we just pass them through as labelled rerun points.
class TextArrayPublisher {
 public:
  TextArrayPublisher() = default;
  explicit TextArrayPublisher(std::string path) : path_(std::move(path)) {}
  void publishScores(const std::vector<std::pair<Eigen::Matrix<float, 4, 1>, float>>& objs,
                     const std::array<float, 3>& color) const;

 private:
  std::string path_;
};

}  // namespace erasor2::viz

#endif  // ERASOR2_RERUN_LOGGER_HPP
