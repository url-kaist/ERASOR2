#include "erasor2/RerunLogger.hpp"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>

#include <rerun.hpp>

#include "erasor2/grid_map.hpp"

namespace erasor2::viz {

namespace {

std::unique_ptr<rerun::RecordingStream> g_rec;
std::atomic<bool> g_enabled{false};
std::once_flag g_init_once;

std::array<float, 3> intensityToGrey(float intensity) {
  // Same fallback the legacy publishers relied on — show raw intensity as
  // greyscale. Clamp because PCL clouds carry arbitrary values.
  float v = std::min(1.0f, std::max(0.0f, intensity / 255.0f));
  return {v, v, v};
}

}  // namespace

void init(const std::string& app_id, bool spawn, const std::string& save_path) {
  std::call_once(g_init_once, [&]() {
    g_rec = std::make_unique<rerun::RecordingStream>(app_id);
    if (!save_path.empty()) {
      g_rec->save(save_path).exit_on_failure();
    } else if (spawn) {
      g_rec->spawn().exit_on_failure();
    } else {
      g_rec->connect_grpc().exit_on_failure();
    }
    g_enabled.store(true);
  });
}

void shutdown() {
  g_enabled.store(false);
  if (g_rec) {
    g_rec->flush_blocking();
    g_rec.reset();
  }
}

void setEnabled(bool enabled) { g_enabled.store(enabled); }
bool isEnabled() { return g_enabled.load() && g_rec != nullptr; }

void setFrame(int64_t frame) {
  if (!isEnabled()) return;
  g_rec->set_time_sequence("frame", frame);
}

void logCloud(std::string_view path,
              const pcl::PointCloud<pcl::PointXYZI>& cloud,
              const std::vector<std::array<float, 3>>& colors) {
  if (!isEnabled() || cloud.empty()) return;
  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud.size());
  for (const auto& p : cloud) {
    positions.emplace_back(rerun::Position3D{p.x, p.y, p.z});
  }
  std::vector<rerun::Color> rerun_colors;
  rerun_colors.reserve(cloud.size());
  if (!colors.empty() && colors.size() == cloud.size()) {
    for (const auto& c : colors) {
      rerun_colors.emplace_back(rerun::Color{static_cast<uint8_t>(std::round(c[0] * 255.0f)),
                                             static_cast<uint8_t>(std::round(c[1] * 255.0f)),
                                             static_cast<uint8_t>(std::round(c[2] * 255.0f))});
    }
  } else {
    for (const auto& p : cloud) {
      auto rgb = intensityToGrey(p.intensity);
      rerun_colors.emplace_back(rerun::Color{static_cast<uint8_t>(std::round(rgb[0] * 255.0f)),
                                             static_cast<uint8_t>(std::round(rgb[1] * 255.0f)),
                                             static_cast<uint8_t>(std::round(rgb[2] * 255.0f))});
    }
  }
  g_rec->log(std::string(path), rerun::Points3D(positions).with_colors(rerun_colors));
}

void logCloud(std::string_view path, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
  if (!isEnabled() || cloud.empty()) return;
  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;
  positions.reserve(cloud.size());
  colors.reserve(cloud.size());
  for (const auto& p : cloud) {
    positions.emplace_back(rerun::Position3D{p.x, p.y, p.z});
    colors.emplace_back(rerun::Color{p.r, p.g, p.b});
  }
  g_rec->log(std::string(path), rerun::Points3D(positions).with_colors(colors));
}

void logPath(std::string_view path, const std::vector<Eigen::Matrix4f>& poses) {
  if (!isEnabled() || poses.empty()) return;
  std::vector<rerun::Position3D> strip;
  strip.reserve(poses.size());
  for (const auto& T : poses) {
    strip.emplace_back(rerun::Position3D{T(0, 3), T(1, 3), T(2, 3)});
  }
  std::vector<std::vector<rerun::Position3D>> strips{strip};
  g_rec->log(std::string(path), rerun::LineStrips3D(strips));
}

void logPose(std::string_view path, const Eigen::Matrix4f& pose) {
  if (!isEnabled()) return;
  Eigen::Matrix3f R = pose.block<3, 3>(0, 0);
  Eigen::Quaternionf q(R);
  rerun::Vec3D t{pose(0, 3), pose(1, 3), pose(2, 3)};
  rerun::Quaternion rq = rerun::Quaternion::from_xyzw(q.x(), q.y(), q.z(), q.w());
  g_rec->log(std::string(path), rerun::Transform3D(t, rq));
}

void logGridMapLayer(std::string_view path, const erasor2::GridMap& gm, const std::string& layer) {
  if (!isEnabled() || !gm.exists(layer)) return;
  const auto& M  = gm.get(layer);  // Eigen::MatrixXf, row-major in grid coords
  const int rows = static_cast<int>(M.rows());
  const int cols = static_cast<int>(M.cols());
  std::vector<uint8_t> img(static_cast<size_t>(rows) * cols);
  // Map matrix values to [0,255] greyscale. NaN → 0.
  float vmin = std::numeric_limits<float>::infinity();
  float vmax = -std::numeric_limits<float>::infinity();
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float v = M(r, c);
      if (std::isfinite(v)) {
        vmin = std::min(vmin, v);
        vmax = std::max(vmax, v);
      }
    }
  }
  if (!std::isfinite(vmin) || vmax <= vmin) {
    vmin = 0.0f;
    vmax = 1.0f;
  }
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float v = M(r, c);
      if (!std::isfinite(v)) {
        img[static_cast<size_t>(r) * cols + c] = 0;
      } else {
        float n = (v - vmin) / (vmax - vmin);
        img[static_cast<size_t>(r) * cols + c] =
            static_cast<uint8_t>(std::round(std::min(1.0f, std::max(0.0f, n)) * 255.0f));
      }
    }
  }
  g_rec->log(
      std::string(path),
      rerun::Image::from_greyscale8(
          img, rerun::WidthHeight(static_cast<uint32_t>(cols), static_cast<uint32_t>(rows))));
}

void logTextAt(std::string_view path,
               const Eigen::Vector3f& position,
               const std::string& text,
               const std::array<float, 3>& color) {
  if (!isEnabled()) return;
  std::vector<rerun::Position3D> pos{rerun::Position3D{position.x(), position.y(), position.z()}};
  std::vector<rerun::Text> labels{rerun::Text(text)};
  rerun::Color rc{static_cast<uint8_t>(std::round(color[0] * 255.0f)),
                  static_cast<uint8_t>(std::round(color[1] * 255.0f)),
                  static_cast<uint8_t>(std::round(color[2] * 255.0f))};
  g_rec->log(std::string(path), rerun::Points3D(pos).with_labels(labels).with_colors({rc}));
}

void TextArrayPublisher::publishScores(
    const std::vector<std::pair<Eigen::Matrix<float, 4, 1>, float>>& objs,
    const std::array<float, 3>& color) const {
  if (!isEnabled() || objs.empty()) return;
  std::vector<rerun::Position3D> pts;
  std::vector<rerun::Text> labels;
  pts.reserve(objs.size());
  labels.reserve(objs.size());
  for (const auto& [pos, score] : objs) {
    pts.emplace_back(rerun::Position3D{pos.x(), pos.y(), pos.z()});
    // `score` is the mean log-odds dyn(S_{k,t}) from Eq. (13) of
    // Lim et al., RSS 2023. The probability that the instance is
    // moving is logit^{-1}(score) = 1 / (1 + exp(-score)) -- the
    // same value that Eq. (15) thresholds against p(p̄_{k,t}). Show
    // the probability up front for at-a-glance reading and keep the
    // raw log-odds in parentheses for debugging.
    const float prob = 1.0f / (1.0f + std::exp(-score));
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.4f (%.2f)", prob, score);
    labels.emplace_back(rerun::Text(buf));
  }
  rerun::Color rc{static_cast<uint8_t>(std::round(color[0] * 255.0f)),
                  static_cast<uint8_t>(std::round(color[1] * 255.0f)),
                  static_cast<uint8_t>(std::round(color[2] * 255.0f))};
  g_rec->log(path_, rerun::Points3D(pts).with_labels(labels).with_colors({rc}));
}

void logCircle(std::string_view path,
               const Eigen::Vector3f& center,
               float radius,
               int num_segments) {
  if (!isEnabled()) return;
  std::vector<rerun::Position3D> circle;
  circle.reserve(num_segments + 1);
  for (int i = 0; i <= num_segments; ++i) {
    float theta = 2.0f * static_cast<float>(M_PI) * i / num_segments;
    circle.emplace_back(rerun::Position3D{
        center.x() + radius * std::cos(theta), center.y() + radius * std::sin(theta), center.z()});
  }
  std::vector<std::vector<rerun::Position3D>> strips{circle};
  g_rec->log(std::string(path), rerun::LineStrips3D(strips));
}

}  // namespace erasor2::viz
