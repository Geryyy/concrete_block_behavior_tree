#pragma once

// The `/cbs/status/*` lines, apart from the nodes that publish them so a test can read
// them without a ROS graph. Same numbers as the node's own RCLCPP_INFO, `key=value` so
// CBSRunPanel does not have to parse prose; a value may hold spaces, trailing only.

#include <cmath>
#include <cstdio>
#include <string>

namespace concrete_block_behavior_tree
{

/// `/cbs/status/inair` -- MeasureBlockFromGripper's result. `pass` counts this node's measurements.
inline std::string inair_status(
  int pass, const std::string & frame, double x, double y, double z, double tilt_rad,
  int samples)
{
  char line[320];
  std::snprintf(
    line, sizeof(line),
    "pass=%d frame=%s x=%.3f y=%.3f z=%.3f tilt_deg=%.2f samples=%d",
    pass, frame.c_str(), x, y, z, tilt_rad * 180.0 / M_PI, samples);
  return std::string(line);
}

/// `/cbs/status/place_pass` -- one SubTreePlaceApproach pass, in the tree's frame (K0). The
/// shift is the miss on a `shift` verdict and nothing otherwise; `off` the total after it.
inline std::string place_pass_status(
  int attempt, int pass, const std::string & verdict, double miss_x, double miss_y,
  double miss_z, double off_x, double off_y)
{
  const bool shift = verdict == "shift";
  char line[320];
  std::snprintf(
    line, sizeof(line),
    "attempt=%d pass=%d verdict=%s miss_x=%.3f miss_y=%.3f miss_z=%.3f shift_x=%.3f "
    "shift_y=%.3f off_x=%.3f off_y=%.3f",
    attempt, pass, verdict.c_str(), miss_x, miss_y, miss_z, shift ? miss_x : 0.0,
    shift ? miss_y : 0.0, off_x + (shift ? miss_x : 0.0), off_y + (shift ? miss_y : 0.0));
  return std::string(line);
}

/// `/cbs/status/pose` -- RunPoseEstimation's outcome; x/y/z the target's pose after it, if listed.
inline std::string pose_status(
  const std::string & mode, const std::string & block, bool ok, const std::string & frame,
  const double * xyz)
{
  char line[320];
  int n = std::snprintf(
    line, sizeof(line), "mode=%s block=%s ok=%d", mode.c_str(), block.c_str(), ok ? 1 : 0);
  if (xyz != nullptr && n > 0 && n < static_cast<int>(sizeof(line))) {
    std::snprintf(
      line + n, sizeof(line) - n, " frame=%s x=%.3f y=%.3f z=%.3f", frame.c_str(), xyz[0],
      xyz[1], xyz[2]);
  }
  return std::string(line);
}

/// `/cbs/status/settle` -- WaitForMoveComplete per tick: the sway gate it applies. `bound` 0
/// means it takes crane_mpc's verdict (`verdict` SwaySettled.SETTLED_*) and not the rates;
/// `still` that gate now, `held`/`hold` its dwell, `msg` what the move still waits for.
inline std::string settle_status(
  double bound, double rate, int verdict, bool still, double held_s, double hold_s,
  double elapsed_s, const std::string & reason)
{
  char line[320];
  std::snprintf(
    line, sizeof(line),
    "bound=%.3f rate=%.3f verdict=%d still=%d held=%.1f hold=%.1f elapsed=%.1f msg=%s",
    bound, rate, verdict, still ? 1 : 0, held_s, hold_s, elapsed_s,
    reason.empty() ? "arrived" : reason.c_str());
  return std::string(line);
}

/// `/cbs/status/load_off` -- WaitForLoadOff per tick. `unload`/`gate` are shares of m g r.
inline std::string load_off_status(
  double effort, double reference, double drop_needed, double unload, double gate,
  double r, double above, bool landed)
{
  char line[320];
  std::snprintf(
    line, sizeof(line),
    "effort=%.0f reference=%.0f drop_needed=%.0f unload=%.3f gate=%.3f r=%.2f "
    "above=%+.3f landed=%d",
    effort, reference, drop_needed, unload, gate, r, above, landed ? 1 : 0);
  return std::string(line);
}

}  // namespace concrete_block_behavior_tree
