#ifndef ERASOR2_PROGRESS_BAR_HPP
#define ERASOR2_PROGRESS_BAR_HPP

// Header-only progress bar inspired by the bar used in HKU-MARS'
// LiDAR_IMU_Init repo. Single carriage-return-rewritten line of the form
//
//   [erasor2] update     [========>             ]  42%  (68/161) ETA 3.2s
//
// Construction:
//   erasor2::ProgressBar bar("[erasor2] update", num_data);
//   for (int k = 0; k < num_data; ++k) {
//     bar.tick(k + 1);
//     ...
//   }
//   bar.finish();
//
// tick(current) is rate-limited so it's safe to call every iteration even
// for tens-of-thousands-of-step loops. finish() emits the 100% line and a
// trailing newline so subsequent log output starts on a fresh row.

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>

namespace erasor2 {

class ProgressBar {
 public:
  ProgressBar(const std::string& label, int total, int bar_width = 30)
      : label_(label),
        total_(total < 1 ? 1 : total),
        bar_width_(bar_width),
        start_(std::chrono::steady_clock::now()),
        last_render_(start_) {}

  // Update the bar. Renders at most ~30 times per second to keep terminal
  // output cheap; pass `force = true` to bypass the rate limit (used by
  // finish()).
  void tick(int current, bool force = false) {
    if (current < 0) current = 0;
    if (current > total_) current = total_;

    const auto now = std::chrono::steady_clock::now();
    const auto since_last =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_render_).count();
    if (!force && since_last < 33 && current != total_) return;
    last_render_ = now;

    const double frac = static_cast<double>(current) / static_cast<double>(total_);
    const int filled = static_cast<int>(frac * bar_width_);

    std::string bar;
    bar.reserve(bar_width_ + 2);
    bar.push_back('[');
    for (int i = 0; i < bar_width_; ++i) {
      if (i < filled - 1)
        bar.push_back('=');
      else if (i == filled - 1)
        bar.push_back('>');
      else
        bar.push_back(' ');
    }
    bar.push_back(']');

    const double elapsed_s =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count() / 1000.0;
    const double eta_s = (current > 0)
                             ? elapsed_s * (static_cast<double>(total_ - current) / current)
                             : 0.0;

    // ANSI cyan for the bar to match the configuration banner style.
    std::printf(
        "\r\033[1;36m%s\033[0m %s %3d%%  (%d/%d) ETA %4.1fs",
        label_.c_str(),
        bar.c_str(),
        static_cast<int>(frac * 100.0),
        current,
        total_,
        eta_s);
    std::fflush(stdout);
  }

  // Render the 100% line and move to a fresh row.
  void finish() {
    tick(total_, /*force=*/true);
    std::printf("\n");
    std::fflush(stdout);
  }

 private:
  std::string label_;
  int total_;
  int bar_width_;
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point last_render_;
};

}  // namespace erasor2

#endif  // ERASOR2_PROGRESS_BAR_HPP
