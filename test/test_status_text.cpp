#include <gtest/gtest.h>

#include <string>

#include "concrete_block_behavior_tree/status_text.hpp"

using concrete_block_behavior_tree::inair_status;
using concrete_block_behavior_tree::load_off_status;
using concrete_block_behavior_tree::pose_status;

TEST(StatusText, inair_carries_what_the_log_line_carries)
{
  // The run2_pp numbers: a 1.2 deg grip read in the planning frame over one pendulum period.
  const std::string line =
    inair_status(2, "K0_mounting_base", 4.1234, -1.5678, 2.0005, 0.0209, 78);
  EXPECT_EQ(
    line,
    "pass=2 frame=K0_mounting_base x=4.123 y=-1.568 z=2.001 tilt_deg=1.20 samples=78");
}

TEST(StatusText, inair_also_carries_the_point_in_the_task_lines_frame)
{
  // The 281 sim: K0_mounting_base is world turned by pi and shifted, block_0's slot
  // world (-15.00, -3.00) is K0 (8.06, 3.35).
  const double world_xyz[3] = {-15.0, -3.0, 0.59};
  EXPECT_NE(
    inair_status(1, "K0_mounting_base", 8.06, 3.35, -0.55, 0.0, 78, world_xyz)
    .find("samples=78 world_x=-15.000 world_y=-3.000 world_z=0.590"), std::string::npos);
}

TEST(StatusText, load_off_carries_the_gate_the_bar_is_drawn_against)
{
  // Run 11: 111 -> 55 kN m on touchdown, 880 kg at r = 5.12 m, gate 0.8 of m g r.
  const std::string line =
    load_off_status(55000.0, 111000.0, 35367.0, 0.63, 0.8, 5.12, -0.004, true);
  EXPECT_EQ(
    line,
    "effort=55000 reference=111000 drop_needed=35367 unload=0.630 gate=0.800 r=5.12 "
    "above=-0.004 landed=1");

  // Mid-descent: nothing off the hook yet, so the bar sits at zero and the flag is clear.
  EXPECT_NE(
    load_off_status(111000.0, 111000.0, 35367.0, 0.0, 0.8, 5.12, 0.312, false)
    .find("unload=0.000 gate=0.800"), std::string::npos);
  EXPECT_NE(
    load_off_status(111000.0, 111000.0, 35367.0, 0.0, 0.8, 5.12, 0.312, false)
    .find("above=+0.312 landed=0"), std::string::npos);
}

TEST(StatusText, pose_carries_the_target_only_when_the_response_listed_it)
{
  const double xyz[3] = {6.0012, 2.0034, 1.05};
  EXPECT_EQ(
    pose_status("REFINE_BLOCK", "block_2", true, "world", xyz),
    "mode=REFINE_BLOCK block=block_2 ok=1 frame=world x=6.001 y=2.003 z=1.050");
  // A timeout: no response, no pose, and the line has to say it failed.
  EXPECT_EQ(
    pose_status("REFINE_GRASPED", "block_2", false, "", nullptr),
    "mode=REFINE_GRASPED block=block_2 ok=0");
}
