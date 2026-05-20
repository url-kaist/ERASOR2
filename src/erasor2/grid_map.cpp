#include "erasor2/grid_map.hpp"

namespace erasor2 {

GridMap::GridMap(const std::vector<std::string>& layers) : layers_(layers) {
  for (const auto& layer : layers_) {
    data_.emplace(layer, Matrix());
  }
}

void GridMap::setGeometry(const Length& length, double resolution, const Position& position) {
  size_(0) = static_cast<int>(std::round(length(0) / resolution));
  size_(1) = static_cast<int>(std::round(length(1) / resolution));
  for (auto& kv : data_) {
    kv.second.resize(size_(0), size_(1));
    kv.second.setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  resolution_ = resolution;
  length_     = (size_.cast<double>() * resolution_).matrix();
  position_   = position;
}

void GridMap::add(const std::string& layer, const Matrix& data) {
  if (data_.find(layer) == data_.end()) {
    layers_.push_back(layer);
  }
  data_[layer] = data;
}

float& GridMap::at(const std::string& layer, const Index& index) {
  auto it = data_.find(layer);
  if (it == data_.end()) {
    throw std::out_of_range("GridMap::at(...): layer '" + layer + "' not present");
  }
  return it->second(index(0), index(1));
}

float GridMap::at(const std::string& layer, const Index& index) const {
  auto it = data_.find(layer);
  if (it == data_.end()) {
    throw std::out_of_range("GridMap::at(...): layer '" + layer + "' not present");
  }
  return it->second(index(0), index(1));
}

Matrix& GridMap::get(const std::string& layer) {
  auto it = data_.find(layer);
  if (it == data_.end()) {
    throw std::out_of_range("GridMap::get(...): layer '" + layer + "' not present");
  }
  return it->second;
}

const Matrix& GridMap::get(const std::string& layer) const {
  auto it = data_.find(layer);
  if (it == data_.end()) {
    throw std::out_of_range("GridMap::get(...): layer '" + layer + "' not present");
  }
  return it->second;
}

bool GridMap::isInside(const Position& position) const {
  // grid_map's checkIfPositionWithinMap: transform = -I applied to
  // (position - center - length/2), then bounds-check vs length.
  const Position t = (position_ + 0.5 * length_) - position;
  return (t.x() >= 0.0 && t.y() >= 0.0 && t.x() < length_(0) && t.y() < length_(1));
}

bool GridMap::getIndex(const Position& position, Index& index) const {
  // Matches grid_map::getIndexFromPosition with bufferStartIndex = (0, 0):
  //   indexVector = (position - 0.5*length - mapPosition) / resolution
  //   index       = -indexVector  (transform map-frame -> buffer order)
  // The negate-then-cast-to-int truncates toward zero, same as upstream.
  if (resolution_ <= 0.0) return false;
  const double dx = (position_(0) + 0.5 * length_(0) - position(0)) / resolution_;
  const double dy = (position_(1) + 0.5 * length_(1) - position(1)) / resolution_;
  index(0)        = static_cast<int>(dx);
  index(1)        = static_cast<int>(dy);
  return isInside(position) &&
         (index(0) >= 0 && index(1) >= 0 && index(0) < size_(0) && index(1) < size_(1));
}

}  // namespace erasor2
