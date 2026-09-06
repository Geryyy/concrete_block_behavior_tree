#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace concrete_block_behavior_tree
{

// The feedforward branch of wiki/control_architecture.md §2, per sample of a planned trajectory.
// Free of ROS and of crane_model so the arithmetic that decides the command can be tested without
// a URDF: everything model-shaped arrives as tau.
//
// Vectors are [sample][joint]; `time` is seconds from the start of the trajectory.
struct FeedforwardPlan
{
  std::vector<double> time;                  // n
  std::vector<std::vector<double>> tau;      // n x m, RNEA along the plan
  std::vector<std::vector<double>> dq_d;     // n x m, planned joint velocity
  std::vector<double> inv_k;                 // m, 1/k_i; 0 on a joint with no C3 fit
  std::vector<double> u_min;                 // m, bound on the feedforward branch
  std::vector<double> u_max;                 // m
  double dead_time_s{0.06};
};

// What the effort field of sample s must carry.
//
//   u_d(t)    = qdot_d(t) + tau_dot_d(t)/k_i          the flat inversion, §5.3
//   u_ff(t)   = u_d(t + dead_time)                     preview: the plan is known ahead of time
//   effort(t) = clamp(u_ff(t)) - qdot_d(t)             the controller adds ff_velocity_scale*qdot_d
//
// Subtracting qdot_d(t) rather than qdot_d(t + dead_time) is what makes the *whole* feedforward
// branch previewed while the feedback branch keeps the un-advanced reference it needs. Carrying
// only tau_dot/k would advance the correction and leave the reference velocity where it was --
// a fifth rung of controller_design.md §4.3's ladder that nothing there scores.
//
// The preview is a *time*, not a sample count: the planners emit different grids (0.01 s on the
// legacy A2B path, 0.04 s on crane_planning) and the dead time is 0.06 s on all of them.
//
// Past the end of the plan the crane is meant to stand still, so the feedforward branch is taken
// to zero rather than held -- which is also what removes the static feedforward over the last
// dead_time of the move, when the axis should already be stopping.
//
// Returns n x m, all zeros on a joint with inv_k == 0 (no C3 fit: the tool axis and the passive
// pair), which then runs the plain static feedforward.
inline std::vector<std::vector<double>> compute_feedforward(
  const FeedforwardPlan & plan, bool * saturated = nullptr)
{
  const size_t n = plan.time.size();
  const size_t m = plan.inv_k.size();
  std::vector<std::vector<double>> effort(n, std::vector<double>(m, 0.0));
  if (n < 2) {
    return effort;
  }

  // tau_dot per sample: central difference, one-sided at the ends. The grid is the planner's and
  // is not guaranteed uniform -- the legacy path appends its final point two steps out -- so
  // divide by the span that was actually used.
  std::vector<std::vector<double>> tau_dot(n, std::vector<double>(m, 0.0));
  for (size_t s = 0; s < n; ++s) {
    const size_t lo = (s == 0) ? 0 : s - 1;
    const size_t hi = (s + 1 == n) ? s : s + 1;
    const double dt = plan.time[hi] - plan.time[lo];
    if (dt <= 0.0) {
      continue;
    }
    for (size_t j = 0; j < m; ++j) {
      tau_dot[s][j] = (plan.tau[hi][j] - plan.tau[lo][j]) / dt;
    }
  }

  // linear interpolation at t, and zero past the end
  const auto sample_at = [&plan, n](const std::vector<std::vector<double>> & values, double t,
      size_t j) {
      if (t >= plan.time[n - 1]) {
        return 0.0;
      }
      if (t <= plan.time[0]) {
        return values[0][j];
      }
      const auto upper = std::upper_bound(plan.time.begin(), plan.time.end(), t);
      const size_t hi = static_cast<size_t>(std::distance(plan.time.begin(), upper));
      const size_t lo = hi - 1;
      const double span = plan.time[hi] - plan.time[lo];
      if (span <= 0.0) {
        return values[lo][j];
      }
      const double alpha = (t - plan.time[lo]) / span;
      return values[lo][j] * (1.0 - alpha) + values[hi][j] * alpha;
    };

  for (size_t s = 0; s < n; ++s) {
    const double preview_time = plan.time[s] + plan.dead_time_s;
    for (size_t j = 0; j < m; ++j) {
      if (plan.inv_k[j] == 0.0) {
        continue;
      }
      double u = sample_at(plan.dq_d, preview_time, j) +
        sample_at(tau_dot, preview_time, j) * plan.inv_k[j];
      const double bounded = std::min(std::max(u, plan.u_min[j]), plan.u_max[j]);
      if (bounded != u && saturated != nullptr) {
        *saturated = true;
      }
      effort[s][j] = bounded - plan.dq_d[s][j];
    }
  }
  return effort;
}

// qddot along the plan, when the planner does not send it. Both A2B servers deliberately omit the
// acceleration field ("don't respond acceleration to avoid effects from interpolation",
// a2b_server_base.cpp), so without this the feedforward has no tau_d to build on.
//
// Differentiating the planned velocity costs one derivative of smoothness: the inversion already
// needs qdddot_d, so tau_dot then rests on a second difference of qdot_d. On a segment boundary
// that is a staircase, which is the C^3/C^4 requirement of controller_design.md §2.4 showing up as
// noise rather than as an error.
inline std::vector<std::vector<double>> differentiate(
  const std::vector<double> & time, const std::vector<std::vector<double>> & values)
{
  const size_t n = time.size();
  std::vector<std::vector<double>> derivative(n);
  for (size_t s = 0; s < n; ++s) {
    derivative[s].assign(values[s].size(), 0.0);
    const size_t lo = (s == 0) ? 0 : s - 1;
    const size_t hi = (s + 1 == n) ? s : s + 1;
    const double dt = time[hi] - time[lo];
    if (dt <= 0.0) {
      continue;
    }
    for (size_t j = 0; j < values[s].size(); ++j) {
      derivative[s][j] = (values[hi][j] - values[lo][j]) / dt;
    }
  }
  return derivative;
}

}  // namespace concrete_block_behavior_tree
