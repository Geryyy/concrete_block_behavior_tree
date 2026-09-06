// The arithmetic of the C3 feedforward, without ROS, a URDF or crane_model. Everything
// model-shaped enters as tau, so what is under test is the part that decides the command:
// the preview, the branch split and the bound.

#include <cmath>
#include <limits>
#include <vector>

#include "concrete_block_behavior_tree/c3_feedforward_math.hpp"
#include "gtest/gtest.h"

using concrete_block_behavior_tree::FeedforwardPlan;
using concrete_block_behavior_tree::compute_feedforward;
using concrete_block_behavior_tree::differentiate;

namespace
{

constexpr double kDeadTime = 0.06;
constexpr double kInvK = 0.5;

// one joint, constant acceleration `a` and constant tau rate `tau_rate`, sampled on `dt`
FeedforwardPlan ramp_plan(double dt, size_t n, double a, double tau_rate)
{
  FeedforwardPlan plan;
  plan.dead_time_s = kDeadTime;
  plan.inv_k = {kInvK};
  plan.u_min = {-std::numeric_limits<double>::infinity()};
  plan.u_max = {std::numeric_limits<double>::infinity()};
  for (size_t s = 0; s < n; ++s) {
    const double t = static_cast<double>(s) * dt;
    plan.time.push_back(t);
    plan.dq_d.push_back({a * t});
    plan.tau.push_back({tau_rate * t});
  }
  return plan;
}

}  // namespace

// A plan the inversion has nothing to correct: constant velocity, constant tau. The feedforward
// must be exactly zero, not a residual of the differentiation.
TEST(C3FeedforwardMath, ConstantVelocityNeedsNoCorrection)
{
  auto plan = ramp_plan(0.01, 100, 0.0, 0.0);
  for (auto & point : plan.dq_d) {
    point[0] = 0.3;
  }

  const auto effort = compute_feedforward(plan);
  for (size_t s = 0; s + 10 < effort.size(); ++s) {
    EXPECT_NEAR(effort[s][0], 0.0, 1e-12) << "sample " << s;
  }
}

// The whole feedforward branch is advanced, not only the tau_dot correction. With tau constant the
// command must still lead the reference by the velocity the plan reaches one dead time later --
// which is the half-advance the effort field would carry if it held tau_dot/k alone.
TEST(C3FeedforwardMath, TheReferenceVelocityIsAdvancedToo)
{
  constexpr double a = 0.4;  // rad/s^2
  const auto plan = ramp_plan(0.01, 200, a, 0.0);

  const auto effort = compute_feedforward(plan);
  for (size_t s = 10; s + 20 < effort.size(); ++s) {
    EXPECT_NEAR(effort[s][0], a * kDeadTime, 1e-9) << "sample " << s;
  }
}

// The preview is a time. The same plan sampled on the legacy A2B grid (0.01 s) and on
// crane_planning's (0.04 s) must produce the same command at the same instant; counting samples
// instead would advance one of them by four times the dead time.
TEST(C3FeedforwardMath, ThePreviewIsSecondsNotSamples)
{
  constexpr double a = 0.4;
  constexpr double tau_rate = 120.0;
  const auto fine = compute_feedforward(ramp_plan(0.01, 200, a, tau_rate));
  const auto coarse = compute_feedforward(ramp_plan(0.04, 50, a, tau_rate));

  // t = 0.4 s is sample 40 on the fine grid and sample 10 on the coarse one
  EXPECT_NEAR(fine[40][0], coarse[10][0], 1e-9);
  // and it is not zero, so the agreement is not the trivial one
  EXPECT_GT(std::abs(fine[40][0]), 1.0);
}

// tau_dot/k enters with the sign and scale of the flat inversion u = qdot_d + tau_dot_d/k_i.
TEST(C3FeedforwardMath, TheCorrectionIsTauDotOverK)
{
  constexpr double tau_rate = 120.0;  // N m/s
  const auto plan = ramp_plan(0.01, 200, 0.0, tau_rate);

  const auto effort = compute_feedforward(plan);
  for (size_t s = 10; s + 20 < effort.size(); ++s) {
    EXPECT_NEAR(effort[s][0], tau_rate * kInvK, 1e-9) << "sample " << s;
  }
}

// Nothing downstream bounds the feedforward branch, so it is bounded here. The bound is on the
// command u, not on the effort field, because it is u that Psi has to evaluate.
TEST(C3FeedforwardMath, TheFeedforwardBranchIsBoundedAtItsSource)
{
  auto plan = ramp_plan(0.01, 200, 0.0, 1.0e5);
  plan.u_max = {0.9};
  plan.u_min = {-0.9};
  for (auto & point : plan.dq_d) {
    point[0] = 0.2;
  }

  bool saturated = false;
  const auto effort = compute_feedforward(plan, &saturated);

  EXPECT_TRUE(saturated);
  for (size_t s = 10; s + 20 < effort.size(); ++s) {
    // u = effort + qdot_d(t) sits exactly on the bound
    EXPECT_NEAR(effort[s][0] + plan.dq_d[s][0], 0.9, 1e-9) << "sample " << s;
  }
}

// Over the last dead time the plan has run out. The feedforward branch goes to zero there rather
// than holding its last value, so the command stops leading a reference that has ended.
TEST(C3FeedforwardMath, TheFeedforwardEndsWithThePlan)
{
  const auto plan = ramp_plan(0.01, 100, 0.0, 0.0);
  const auto effort = compute_feedforward(plan);

  // the last sample is one whole dead time past the end of the plan
  EXPECT_NEAR(effort.back()[0] + plan.dq_d.back()[0], 0.0, 1e-12);
}

// A joint with no C3 fit gets no correction at all, whatever the plan does; it keeps running the
// plain static feedforward.
TEST(C3FeedforwardMath, AJointWithoutAFitIsLeftAlone)
{
  FeedforwardPlan plan = ramp_plan(0.01, 50, 0.4, 120.0);
  plan.inv_k = {0.0};

  const auto effort = compute_feedforward(plan);
  for (const auto & point : effort) {
    EXPECT_EQ(point[0], 0.0);
  }
}

// The planners send no accelerations, so qddot is a difference of the planned velocity. Central
// difference on the interior, one-sided at the ends, and correct across a non-uniform grid -- the
// legacy path stamps its final point two steps out.
TEST(C3FeedforwardMath, DifferentiateHandlesANonUniformGrid)
{
  const std::vector<double> time{0.0, 0.01, 0.02, 0.04};
  const std::vector<std::vector<double>> values{{0.0}, {0.1}, {0.2}, {0.4}};

  const auto derivative = differentiate(time, values);

  EXPECT_NEAR(derivative[0][0], 10.0, 1e-12);  // one-sided
  EXPECT_NEAR(derivative[1][0], 10.0, 1e-12);  // central
  EXPECT_NEAR(derivative[2][0], 10.0, 1e-12);  // central over 0.01 + 0.02
  EXPECT_NEAR(derivative[3][0], 10.0, 1e-12);  // one-sided over the doubled final step
}
