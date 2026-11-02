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
