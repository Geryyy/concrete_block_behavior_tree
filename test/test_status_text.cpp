#include <gtest/gtest.h>

#include <string>

#include "concrete_block_behavior_tree/status_text.hpp"

using concrete_block_behavior_tree::inair_status;
using concrete_block_behavior_tree::load_off_status;

TEST(StatusText, inair_carries_what_the_log_line_carries)
{
  // The run2_pp numbers: a 1.2 deg grip read in the planning frame over one pendulum period.
  const std::string line =
    inair_status(2, "K0_mounting_base", 4.1234, -1.5678, 2.0005, 0.0209, 78);
  EXPECT_EQ(
    line,
    "pass=2 frame=K0_mounting_base x=4.123 y=-1.568 z=2.001 tilt_deg=1.20 samples=78");
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
