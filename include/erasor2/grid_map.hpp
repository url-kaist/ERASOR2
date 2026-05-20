// Self-contained replacement for the subset of anybotics/grid_map used by
// ERASOR2. Math (index <-> position) and storage semantics mirror upstream
// grid_map 1.6.4 with buffer start always at (0, 0) — we never call
// GridMap::move(), so the circular-buffer machinery upstream carries is
// inert for us and is omitted.
//
// What we keep: ctor(layers), setGeometry/setPosition/setFrameId, at,
// operator[], get, exists, getIndex, isInside, getSize/getResolution/
// getLength/getPosition, plus GridMapCvConverter::{toImage,addLayerFromImage}
// for the <unsigned char, 1> specialization that dilateAndErode uses.

#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace erasor2 {

using Index    = Eigen::Array2i;
using Size     = Eigen::Array2i;
using Position = Eigen::Vector2d;
using Length   = Eigen::Vector2d;
using Matrix   = Eigen::MatrixXf;

class GridMap {
 public:
  GridMap() = default;
  explicit GridMap(const std::vector<std::string>& layers);

  void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
  const std::string& getFrameId() const { return frame_id_; }

  // Mirrors grid_map::GridMap::setGeometry: rounds length/resolution to an
  // integer cell count, then snaps length_ back to size_*resolution.
  void setGeometry(const Length& length,
                   double resolution,
                   const Position& position = Position::Zero());
  void setPosition(const Position& position) { position_ = position; }

  void add(const std::string& layer, const Matrix& data);

  float& at(const std::string& layer, const Index& index);
  float at(const std::string& layer, const Index& index) const;

  Matrix& operator[](const std::string& layer) { return get(layer); }
  const Matrix& operator[](const std::string& layer) const { return get(layer); }
  Matrix& get(const std::string& layer);
  const Matrix& get(const std::string& layer) const;

  bool exists(const std::string& layer) const { return data_.count(layer) != 0; }

  bool getIndex(const Position& position, Index& index) const;
  bool isInside(const Position& position) const;

  const Size& getSize() const { return size_; }
  double getResolution() const { return resolution_; }
  const Length& getLength() const { return length_; }
  const Position& getPosition() const { return position_; }
  const std::vector<std::string>& getLayers() const { return layers_; }

 private:
  std::unordered_map<std::string, Matrix> data_;
  std::vector<std::string> layers_;
  std::string frame_id_;
  Position position_ = Position::Zero();
  Length length_     = Length::Zero();
  Size size_         = Size::Zero();
  double resolution_ = 0.0;
};

// Minimal port of grid_map_cv::GridMapCvConverter. Only the
// <unsigned char, 1> specialization is needed for dilateAndErode /
// erodeGridMap. Generic enough to swap in another integer Type_ if a future
// pass needs CV_16UC1, etc.
class GridMapCvConverter {
 public:
  template <typename Type_, int NChannels_>
  static bool toImage(const GridMap& gm,
                      const std::string& layer,
                      int encoding,
                      float lower_value,
                      float upper_value,
                      cv::Mat& image);

  template <typename Type_, int NChannels_>
  static bool addLayerFromImage(const cv::Mat& image,
                                const std::string& layer,
                                GridMap& gm,
                                float lower_value,
                                float upper_value);
};

template <typename Type_, int NChannels_>
bool GridMapCvConverter::toImage(const GridMap& gm,
                                 const std::string& layer,
                                 int encoding,
                                 float lower_value,
                                 float upper_value,
                                 cv::Mat& image) {
  const Size& s   = gm.getSize();
  image           = cv::Mat::zeros(s(0), s(1), encoding);
  const Matrix& M = gm.get(layer);
  if (M.rows() != s(0) || M.cols() != s(1)) return false;
  const float range = upper_value - lower_value;
  if (range == 0.0f) return false;
  const float max_t = static_cast<float>(std::numeric_limits<Type_>::max());
  for (int r = 0; r < s(0); ++r) {
    for (int c = 0; c < s(1); ++c) {
      const float v = M(r, c);
      if (std::isfinite(v)) {
        image.at<Type_>(r, c) = static_cast<Type_>((v - lower_value) / range * max_t);
      }
      // non-finite -> stays 0 (no-data convention from upstream)
    }
  }
  return true;
}

template <typename Type_, int NChannels_>
bool GridMapCvConverter::addLayerFromImage(const cv::Mat& image,
                                           const std::string& layer,
                                           GridMap& gm,
                                           float lower_value,
                                           float upper_value) {
  const Size& s = gm.getSize();
  if (image.rows != s(0) || image.cols != s(1)) return false;
  const float range = upper_value - lower_value;
  const float max_t = static_cast<float>(std::numeric_limits<Type_>::max());
  Matrix data(s(0), s(1));
  for (int r = 0; r < s(0); ++r) {
    for (int c = 0; c < s(1); ++c) {
      const float img_v = static_cast<float>(image.at<Type_>(r, c));
      data(r, c)        = lower_value + (img_v / max_t) * range;
    }
  }
  gm.add(layer, data);
  return true;
}

}  // namespace erasor2
