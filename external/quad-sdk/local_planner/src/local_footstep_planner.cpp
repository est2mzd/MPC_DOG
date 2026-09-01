#include "local_planner/local_footstep_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>

namespace {

// [MPC_DOG shadow instrumentation] Measurement-only. Enabled iff env
// MPCDOG_STEPDUMP_DIR (or the legacy MPCDOG_STEP09_DIR) is set and non-empty.
// Writes CSVs used by the Step 09+ analyses (terrain-map layers, per-touchdown
// foothold selection, future gait events, ...). Adds NOTHING to the control
// path: no return value, cmd_vel, foothold or NMPC input changes.
const char *stepDumpDir() {
  static const char *d = []() {
    const char *v = std::getenv("MPCDOG_STEPDUMP_DIR");
    if (v == nullptr || v[0] == '\0') v = std::getenv("MPCDOG_STEP09_DIR");
    return v;
  }();
  return (d != nullptr && d[0] != '\0') ? d : nullptr;
}

double step09Sample(const grid_map::GridMap &grid, const char *layer, double x,
                    double y) {
  const grid_map::Position p(x, y);
  if (!grid.exists(layer) || !grid.isInside(p)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return grid.atPosition(layer, p);
}

// Dump the terrain-map row nearest y = 0 (the robot's path; the test trenches
// are full-width in y so any row is representative). Written once per process.
void step09DumpMapCrossSection(const grid_map::GridMap &grid,
                               double obj_threshold, const std::string &dir) {
  std::ofstream f(dir + "/step09_map_cross_section.csv");
  if (!f) return;
  f << "map_stamp,frame,cell_i,cell_j,x,y,z_raw,z_inpainted,z_smooth,slope,"
       "roughness,hole_mask_recon,hole_mask_filtered,traversability,"
       "traversability_mask,observed,inside_map,binary_safe\n";
  const long stamp = static_cast<long>(grid.getTimestamp());
  const std::string frame = grid.getFrameId();
  const grid_map::Size sz = grid.getSize();
  if (sz(0) == 0 || sz(1) == 0) return;
  grid_map::Index c0;
  int jc = grid.getIndex(grid_map::Position(0.0, 0.0), c0) ? c0(1) : sz(1) / 2;
  auto lyr = [&](const char *n) -> const grid_map::Matrix * {
    return grid.exists(n) ? &grid.get(n) : nullptr;
  };
  const grid_map::Matrix *Lz = lyr("z");
  const grid_map::Matrix *Lzi = lyr("z_inpainted");
  const grid_map::Matrix *Lzs = lyr("z_smooth");
  const grid_map::Matrix *Lsl = lyr("slope");
  const grid_map::Matrix *Lro = lyr("roughness");
  const grid_map::Matrix *Ltr = lyr("traversability");
  const grid_map::Matrix *Ltm = lyr("traversability_mask");
  for (int i = 0; i < sz(0); ++i) {
    grid_map::Position pos;
    grid.getPosition(grid_map::Index(i, jc), pos);
    const double z = Lz ? (*Lz)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    const double zi =
        Lzi ? (*Lzi)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    const double zs =
        Lzs ? (*Lzs)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    const double sl =
        Lsl ? (*Lsl)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    const double ro =
        Lro ? (*Lro)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    const double tr =
        Ltr ? (*Ltr)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    const double tm =
        Ltm ? (*Ltm)(i, jc) : std::numeric_limits<double>::quiet_NaN();
    // hole_mask is deleted by filter_chain.yaml (filter16) before publish;
    // reconstruct 1 - |z - z_inpainted| clamped to [0,1]. hole_mask_filtered
    // (the 0.075 m mean) is NOT recoverable -> emitted as nan.
    double hm = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(z) && std::isfinite(zi)) {
      hm = std::max(0.0, std::min(1.0, 1.0 - std::abs(z - zi)));
    }
    const int observed = std::isfinite(z) ? 1 : 0;
    const int safe = (std::isfinite(tr) && tr > obj_threshold) ? 1 : 0;
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "%ld,%s,%d,%d,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,nan,"
                  "%.5f,%.5f,%d,1,%d\n",
                  stamp, frame.c_str(), i, jc, pos.x(), pos.y(), z, zi, zs, sl,
                  ro, hm, tr, tm, observed, safe);
    f << buf;
  }
}

// [MPC_DOG Step 11] Measurement-only: for one future touchdown, enumerate the
// terrain-map cells that are (a) inside the leg's reach - the SAME 3D distance
// test Phase 4 uses, `||cell - hip|| <= ik_max_reach`, plus a coarse fore/aft/
// lateral box so it is not "spherical only" (instruction 2.3-6), (b) safe
// (traversability > threshold) with all 4 orthogonal neighbours also safe
// (foot-sole / edge margin), (c) observed (raw z finite; unknown != safe).
// Appends one summary row per call. Nothing here feeds control.
void step11EnumerateCandidates(const grid_map::GridMap &grid,
                               const char *trav_layer, double obj_threshold,
                               double toe_radius, const Eigen::Vector3d &hip,
                               double ik_max_reach, double time_s, int plan_idx,
                               const char *leg, size_t td_idx, double sel_x,
                               double sel_y, const std::string &dir) {
  const double R = (ik_max_reach > 0.0) ? ik_max_reach : 0.45;
  const double res = std::max(grid.getResolution(), 1e-3);
  // coarse go2 leg workspace box around the hip (documented as approximate)
  const double fwd = 0.32, back = 0.28, lat = 0.26;
  const bool have_z = grid.exists("z_inpainted");
  const bool have_raw = grid.exists("z");
  const bool have_tr = grid.exists(trav_layer);

  auto trav_ok = [&](double x, double y) -> bool {
    const grid_map::Position p(x, y);
    if (!have_tr || !grid.isInside(p)) return false;
    const double t = grid.atPosition(trav_layer, p);
    return std::isfinite(t) && t > obj_threshold;
  };

  int n_reach = 0, n_safe = 0, n_valid = 0;
  double min_valid_reach = std::numeric_limits<double>::quiet_NaN();
  for (double x = hip.x() - back; x <= hip.x() + fwd + 1e-9; x += res) {
    for (double y = hip.y() - lat; y <= hip.y() + lat + 1e-9; y += res) {
      const grid_map::Position p(x, y);
      if (!grid.isInside(p)) continue;
      const double zc =
          have_z ? grid.atPosition("z_inpainted", p) + toe_radius : 0.0;
      const double d = std::sqrt((x - hip.x()) * (x - hip.x()) +
                                 (y - hip.y()) * (y - hip.y()) +
                                 (zc - hip.z()) * (zc - hip.z()));
      if (d > R) continue;
      ++n_reach;
      if (!trav_ok(x, y)) continue;
      ++n_safe;
      const bool sole = trav_ok(x + res, y) && trav_ok(x - res, y) &&
                        trav_ok(x, y + res) && trav_ok(x, y - res);
      const double zr = have_raw ? grid.atPosition("z", p)
                                 : std::numeric_limits<double>::quiet_NaN();
      const bool observed = std::isfinite(zr);
      if (sole && observed) {
        ++n_valid;
        if (!(min_valid_reach == min_valid_reach) || d < min_valid_reach)
          min_valid_reach = d;
      }
    }
  }

  // does the actually-selected foothold pass the same tests?
  int sel_reach = 0, sel_all = 0;
  if (std::isfinite(sel_x) && std::isfinite(sel_y)) {
    const grid_map::Position sp(sel_x, sel_y);
    double szc =
        (have_z && grid.isInside(sp))
            ? grid.atPosition("z_inpainted", sp) + toe_radius
            : 0.0;
    const double sd = std::sqrt((sel_x - hip.x()) * (sel_x - hip.x()) +
                                (sel_y - hip.y()) * (sel_y - hip.y()) +
                                (szc - hip.z()) * (szc - hip.z()));
    sel_reach = (sd <= R) ? 1 : 0;
    const double szr = (have_raw && grid.isInside(sp)) ? grid.atPosition("z", sp)
                                                      : std::nan("");
    sel_all = (sel_reach && trav_ok(sel_x, sel_y) &&
              trav_ok(sel_x + res, sel_y) && trav_ok(sel_x - res, sel_y) &&
              trav_ok(sel_x, sel_y + res) && trav_ok(sel_x, sel_y - res) &&
              std::isfinite(szr))
                 ? 1
                 : 0;
  }

  const std::string path = dir + "/step11_candidates.csv";
  const bool hdr = !std::ifstream(path).good();
  std::ofstream f(path, std::ios::app);
  if (!f) return;
  if (hdr) {
    f << "time,current_plan_index,leg,touchdown_index,hip_x,hip_y,hip_z,"
         "n_in_reach,n_safe,n_valid,min_valid_reach_dist,sel_x,sel_y,"
         "sel_in_reach,sel_passes_all,ik_max_reach\n";
  }
  char b[384];
  std::snprintf(b, sizeof(b),
                "%.4f,%d,%s,%zu,%.5f,%.5f,%.5f,%d,%d,%d,%.5f,%.5f,%.5f,%d,%d,%.3f",
                time_s, plan_idx, leg, td_idx, hip.x(), hip.y(), hip.z(), n_reach,
                n_safe, n_valid, min_valid_reach, sel_x, sel_y, sel_reach,
                sel_all, R);
  f << b << "\n";
}

// [MPC_DOG Step 12] Measurement-only, shadow. A width-1 greedy multi-step
// foothold search along the reconstructed crawl touchdown order (FL,BR,FR,BL).
// At each future touchdown it projects the body forward at v_fwd, gets that
// leg's hip, enumerates reachable+safe+observed cells (same tests as Step 11),
// and takes the min-cost one. Verdict:
//   FEASIBLE_TO_RANGE     - placed footholds out to plan_distance (or the step cap)
//   BLOCKED_AT_STEP_K     - no valid candidate at touchdown k
//   UNKNOWN_BEFORE_RANGE  - ran off the mapped area first
// Nothing here feeds control. Throttled to every 5th plan cycle.
void step12PlanSequence(const grid_map::GridMap &grid,
                        const std::shared_ptr<quad_utils::QuadKD2> &kd, double dt,
                        int period, double bx, double by, double byaw, double bz,
                        double v_fwd, double ik_max_reach, double toe_radius,
                        double obj_threshold, double plan_distance, double time_s,
                        int plan_idx, const std::string &dir) {
  const auto t_start = std::chrono::steady_clock::now();
  const char *legname[4] = {"FL", "BL", "FR", "BR"};
  const int order[4] = {0, 3, 2, 1};  // observed crawl touchdown order
  const double R = (ik_max_reach > 0.0) ? ik_max_reach : 0.45;
  const double res = std::max(grid.getResolution(), 1e-3);
  const double fwd = 0.32, back = 0.28, lat = 0.26;
  const double td_spacing = (period > 0) ? (period * dt / 4.0) : 0.225;
  const int kmax = 32;
  // Max forward travel of one leg between its own consecutive touchdowns
  // (instruction 2.3-8). go2 crawl is ~0.15-0.25 m; allow a little slack.
  const double max_step_fwd = 0.30, max_step_back = 0.15;
  double leg_prev_x[4] = {-1e9, -1e9, -1e9, -1e9};

  auto trav_ok = [&](double x, double y) -> bool {
    const grid_map::Position p(x, y);
    if (!grid.exists("traversability") || !grid.isInside(p)) return false;
    const double t = grid.atPosition("traversability", p);
    return std::isfinite(t) && t > obj_threshold;
  };
  const bool have_z = grid.exists("z_inpainted");
  const bool have_raw = grid.exists("z");

  std::string verdict = "FEASIBLE_TO_RANGE";
  int blocked_k = -1;
  const char *blocked_leg = "-";
  double max_progress = 0.0;
  int placed = 0;
  std::vector<std::string> foot_rows;

  double prev_x = bx, prev_y = by;
  for (int k = 0; k < kmax; ++k) {
    const int leg = order[k % 4];
    const double t = (k + 1) * td_spacing;
    const double bxk = bx + v_fwd * t;
    Eigen::Vector3d hip;
    kd->worldToNominalHipFKWorldFrame(leg, Eigen::Vector3d(bxk, by, bz),
                                     Eigen::Vector3d(0.0, 0.0, byaw), hip);
    if (!grid.isInside(grid_map::Position(hip.x(), hip.y()))) {
      verdict = "UNKNOWN_BEFORE_RANGE";
      blocked_k = k;
      blocked_leg = legname[leg];
      break;
    }
    // enumerate candidates, keep min-cost valid one
    double best_cost = std::numeric_limits<double>::max();
    double best_x = 0.0, best_y = 0.0;
    int n_valid = 0;
    const double nom_x = hip.x() + 0.08;  // slight forward of the hip
    for (double x = hip.x() - back; x <= hip.x() + fwd + 1e-9; x += res) {
      for (double y = hip.y() - lat; y <= hip.y() + lat + 1e-9; y += res) {
        const grid_map::Position p(x, y);
        if (!grid.isInside(p)) continue;
        const double zc =
            have_z ? grid.atPosition("z_inpainted", p) + toe_radius : 0.0;
        const double d = std::sqrt((x - hip.x()) * (x - hip.x()) +
                                   (y - hip.y()) * (y - hip.y()) +
                                   (zc - hip.z()) * (zc - hip.z()));
        if (d > R || !trav_ok(x, y)) continue;
        // step-length limit vs this leg's previous foothold in the sequence
        if (leg_prev_x[leg] > -1e8 &&
            (x - leg_prev_x[leg] > max_step_fwd ||
             x - leg_prev_x[leg] < -max_step_back)) {
          continue;
        }
        const bool sole = trav_ok(x + res, y) && trav_ok(x - res, y) &&
                          trav_ok(x, y + res) && trav_ok(x, y - res);
        const double zr = have_raw ? grid.atPosition("z", p) : std::nan("");
        if (!sole || !std::isfinite(zr)) continue;
        ++n_valid;
        const double cost = std::fabs(x - nom_x) + std::fabs(y - hip.y()) +
                            0.5 * std::fabs(y - prev_y) + 0.3 * (d / R);
        if (cost < best_cost) {
          best_cost = cost;
          best_x = x;
          best_y = y;
        }
      }
    }
    if (n_valid == 0) {
      verdict = "BLOCKED_AT_STEP_K";
      blocked_k = k;
      blocked_leg = legname[leg];
      break;
    }
    ++placed;
    max_progress = best_x - bx;
    prev_x = best_x;
    prev_y = best_y;
    leg_prev_x[leg] = best_x;
    char fb[192];
    std::snprintf(fb, sizeof(fb), "%.4f,%d,%d,%s,%.5f,%.5f,%.5f,%d", time_s,
                  plan_idx, k, legname[leg], best_x, best_y, hip.x(), n_valid);
    foot_rows.emplace_back(fb);
    if (max_progress >= plan_distance) {
      verdict = "FEASIBLE_TO_RANGE";
      break;
    }
  }

  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t_start)
                      .count();

  const std::string sp = dir + "/step12_sequence.csv";
  const bool sh = !std::ifstream(sp).good();
  std::ofstream sf(sp, std::ios::app);
  if (sf) {
    if (sh) {
      sf << "time,current_plan_index,verdict,blocked_step_k,blocked_leg,"
            "n_placed,max_feasible_progress_m,plan_distance_m,compute_time_us\n";
    }
    char b[256];
    std::snprintf(b, sizeof(b), "%.4f,%d,%s,%d,%s,%d,%.4f,%.2f,%ld", time_s,
                  plan_idx, verdict.c_str(), blocked_k, blocked_leg, placed,
                  max_progress, plan_distance, static_cast<long>(us));
    sf << b << "\n";
  }
  // planned foothold sequence: only every 20th call, to bound size
  static long s12_foot_calls = 0;
  if ((s12_foot_calls++ % 20) == 0 && !foot_rows.empty()) {
    const std::string fp = dir + "/step12_footholds.csv";
    const bool fh = !std::ifstream(fp).good();
    std::ofstream ff(fp, std::ios::app);
    if (ff) {
      if (fh) ff << "time,current_plan_index,step_k,leg,x,y,hip_x,n_valid\n";
      for (const auto &r : foot_rows) ff << r << "\n";
    }
  }
}

}  // namespace

LocalFootstepPlanner::LocalFootstepPlanner(rclcpp::Node::SharedPtr node)
    : node_(node) {}

void LocalFootstepPlanner::setTemporalParams(
    double dt, int period, int horizon_length,
    const std::vector<double>& duty_cycles,
    const std::vector<double>& phase_offsets) {
  dt_ = dt;
  period_ = period;
  horizon_length_ = horizon_length;
  duty_cycles_ = duty_cycles;
  phase_offsets_ = phase_offsets;

  nominal_contact_schedule_.resize(period_);

  // Build one gait-period contact table from duty cycles and phase offsets.
  for (int i = 0; i < period_; i++) {
    nominal_contact_schedule_.at(i).resize(num_feet_);
    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) {
      if ((i >= period_ * phase_offsets_[leg_idx] &&
           i < period_ * (phase_offsets_[leg_idx] + duty_cycles_[leg_idx])) ||
          i < period_ *
                  (phase_offsets_[leg_idx] + duty_cycles_[leg_idx] - 1.0)) {
        nominal_contact_schedule_.at(i).at(leg_idx) = true;
      } else {
        nominal_contact_schedule_.at(i).at(leg_idx) = false;
      }
    }
  }
}

void LocalFootstepPlanner::setSpatialParams(
    double ground_clearance, double hip_clearance, double grf_weight,
    double standing_error_threshold,
    std::shared_ptr<quad_utils::QuadKD2> kinematics,
    double foothold_search_radius, double foothold_obj_threshold,
    std::string obj_fun_layer, double toe_radius, double edge_clearance,
    double max_crossable_gap, bool ik_reach_check, double ik_max_reach) {
  ground_clearance_ = ground_clearance;
  hip_clearance_ = hip_clearance;
  standing_error_threshold_ = standing_error_threshold;
  grf_weight_ = grf_weight;
  quadKD_ = kinematics;
  foothold_search_radius_ = foothold_search_radius;
  foothold_obj_threshold_ = foothold_obj_threshold;
  obj_fun_layer_ = obj_fun_layer;
  toe_radius_ = toe_radius;
  edge_clearance_ = edge_clearance;
  max_crossable_gap_ = max_crossable_gap;
  ik_reach_check_ = ik_reach_check;
  ik_max_reach_ = ik_max_reach;
}

void LocalFootstepPlanner::updateMap(const FastTerrainMap& terrain) {
  terrain_ = terrain;
}

void LocalFootstepPlanner::updateMap(const grid_map::GridMap& terrain) {
  terrain_grid_ = terrain;
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(
    const Eigen::VectorXd& body_plan,
    const Eigen::VectorXd& foot_positions_world,
    Eigen::VectorXd& foot_positions_body) {
  for (int i = 0; i < num_feet_; i++) {
    foot_positions_body.segment<3>(3 * i) =
        foot_positions_world.segment<3>(3 * i) - body_plan.segment<3>(0);
  }
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(
    const Eigen::MatrixXd& body_plan,
    const Eigen::MatrixXd& foot_positions_world,
    Eigen::MatrixXd& foot_positions_body) {
  Eigen::VectorXd foot_pos = Eigen::VectorXd::Zero(3 * num_feet_);
  for (int i = 0; i < horizon_length_; i++) {
    foot_pos.setZero();
    getFootPositionsBodyFrame(body_plan.row(i), foot_positions_world.row(i),
                              foot_pos);
    foot_positions_body.row(i) = foot_pos;
  }
}

void LocalFootstepPlanner::computeContactSchedule(
    int current_plan_index, const Eigen::MatrixXd& body_plan,
    const Eigen::VectorXi& ref_primitive_plan, int control_mode,
    std::vector<std::vector<bool>>& contact_schedule) {
  int phase = current_plan_index % period_;

  // Tile the nominal gait from the current phase across the horizon.
  contact_schedule.resize(horizon_length_);
  for (int i = 0; i < horizon_length_; i++) {
    contact_schedule[i].resize(num_feet_);
    if (control_mode == LocalPlannerMode::STAND) {
      for (size_t j = 0; j < contact_schedule[i].size(); j++) {
        contact_schedule[i][j] = true;
      }
    } else {
      contact_schedule[i] = nominal_contact_schedule_[(i + phase) % period_];
    }
  }
  // Override nominal gait during leap, flight, and landing primitives.
  for (int i = 0; i < horizon_length_; i++) {
    if (ref_primitive_plan(i) == LEAP_STANCE) {
      int leading_leg_liftoff_idx = std::min(i, horizon_length_ - 1);

      if (ref_primitive_plan(leading_leg_liftoff_idx) == FLIGHT) {
        contact_schedule.at(i) = {false, true, false, true};
      } else {
        contact_schedule.at(i) = {true, true, true, true};
      }
    } else if (ref_primitive_plan(i) == FLIGHT) {
      std::fill(contact_schedule.at(i).begin(), contact_schedule.at(i).end(),
                false);
    } else if (ref_primitive_plan(i) == LAND_STANCE) {
      contact_schedule.at(i) = {true, true, true, true};
    }
  }
}

void LocalFootstepPlanner::cubicHermiteSpline(double pos_prev, double vel_prev,
                                              double pos_next, double vel_next,
                                              double phase, double duration,
                                              double& pos, double& vel,
                                              double& acc) {
  // Sometimes phase will be slightly smaller than zero due to numerical issues
  phase = std::min(std::max(phase, 0.), 1.);

  double t = phase * duration;
  double t2 = t * t;
  double t3 = t * t * t;
  double duration2 = duration * duration;
  double duration3 = duration * duration * duration;

  pos = pos_prev + vel_prev * t +
        (t3 * (2 * pos_prev - 2 * pos_next + duration * vel_prev +
               duration * vel_next)) /
            duration3 -
        (t2 * (3 * pos_prev - 3 * pos_next + 2 * duration * vel_prev +
               duration * vel_next)) /
            duration2;
  vel = vel_prev +
        (3 * t2 *
         (2 * pos_prev - 2 * pos_next + duration * vel_prev +
          duration * vel_next)) /
            duration3 -
        (2 * t *
         (3 * pos_prev - 3 * pos_next + 2 * duration * vel_prev +
          duration * vel_next)) /
            duration2;
  acc = (6 * t *
         (2 * pos_prev - 2 * pos_next + duration * vel_prev +
          duration * vel_next)) /
            duration3 -
        (2 * (3 * pos_prev - 3 * pos_next + 2 * duration * vel_prev +
              duration * vel_next)) /
            duration2;
}

FootPlanResult LocalFootstepPlanner::computeFootPlan(
    int current_plan_index,
    const std::vector<std::vector<bool>>& contact_schedule,
    const Eigen::MatrixXd& body_plan, const Eigen::MatrixXd& grf_plan,
    const Eigen::MatrixXd& ref_body_plan,
    const Eigen::VectorXd& foot_positions_current,
    const Eigen::VectorXd& foot_velocities_current,
    double first_element_duration,
    quad_msgs::msg::MultiFootState& past_footholds_msg,
    Eigen::MatrixXd& foot_positions, Eigen::MatrixXd& foot_velocities,
    Eigen::MatrixXd& foot_accelerations) {
  // [MPC_DOG DIAG]
  int diag_new_contacts = 0, diag_snap_calls = 0, diag_outside = 0;

  // Phase 2A: track whether every touchdown got a traversable in-map foothold.
  // Records the first failure and a total count; does not alter selection.
  FootPlanResult plan_result;
  auto record_foothold_failure = [&](FootholdStatus status, int leg,
                                     int touchdown_index) {
    if (plan_result.ok) {
      plan_result.ok = false;
      plan_result.worst_status = status;
      plan_result.failed_leg = leg;
      plan_result.failed_touchdown_index = touchdown_index;
    }
    if (plan_result.nearest_failed_index < 0 ||
        touchdown_index < plan_result.nearest_failed_index) {
      plan_result.nearest_failed_index = touchdown_index;
    }
    ++plan_result.failed_count;
  };

  // [MPC_DOG Step 09] measurement-only: collect per-touchdown foothold rows.
  const char *s09_dir = stepDumpDir();
  std::vector<std::string> s09_foot_rows;
  const char *kS09Legs[4] = {"FL", "BL", "FR", "BR"};

  // [MPC_DOG Step 11] enumerate reachable candidates once per leg per cycle.
  bool s11_leg_done[4] = {false, false, false, false};

  // [MPC_DOG Step 10] measurement-only: reconstruct the future touchdown-event
  // list for each leg from the CURRENT gait phase (computeContactSchedule's own
  // output, which already tiles nominal_contact_schedule_ from
  // phase = current_plan_index % period_ - not from phase 0). Shadow only:
  // nothing here feeds control. Recorded once per plan cycle.
  std::vector<std::string> s10_gait_rows;
  if (s09_dir != nullptr) {
    const int phase = (period_ > 0) ? (current_plan_index % period_) : 0;
    for (int j = 0; j < num_feet_; ++j) {
      int found = 0;
      for (size_t i = 1; i < contact_schedule.size() && found < 4; ++i) {
        if (isNewContact(contact_schedule, i, j)) {
          char rb[256];
          std::snprintf(rb, sizeof(rb), "%.4f,%d,%d,%d,%.5f,%s,%d,%zu,%.5f",
                        node_->now().seconds(), current_plan_index, phase,
                        period_, dt_, kS09Legs[j], found, i,
                        (current_plan_index + static_cast<int>(i)) * dt_);
          s10_gait_rows.emplace_back(rb);
          ++found;
        }
      }
    }
  }

  // [MPC_DOG Step 12] shadow multi-step foothold search (every 5th cycle).
  if (s09_dir != nullptr && body_plan.rows() > 1) {
    static long s12_calls = 0;
    if ((s12_calls++ % 5) == 0) {
      const double v_fwd = std::min(
          1.0, std::max(0.0, (body_plan(body_plan.rows() - 1, 0) -
                              body_plan(0, 0)) /
                                 std::max(1e-6, (body_plan.rows() - 1) * dt_)));
      step12PlanSequence(terrain_grid_, quadKD_, dt_, period_, body_plan(0, 0),
                         body_plan(0, 1), body_plan(0, 5), body_plan(0, 2),
                         v_fwd, ik_max_reach_, toe_radius_,
                         foothold_obj_threshold_, 2.5, node_->now().seconds(),
                         current_plan_index, std::string(s09_dir));
    }
  }

  // Place new footholds at touchdown events.
  for (int j = 0; j < num_feet_; j++) {
    for (size_t i = 1; i < contact_schedule.size(); i++) {
      if (isNewContact(contact_schedule, i, j)) {
        ++diag_new_contacts;
        Eigen::Vector3d foot_position_grf, foot_position_nominal,
            hip_position_midstance, centrifugal, vel_tracking,
            foot_position_raibert;

        Eigen::Vector3d body_pos_midstance, body_rpy_midstance,
            body_vel_touchdown, ref_body_vel_touchdown, body_ang_vel_touchdown,
            ref_body_ang_vel_touchdown, grf_midstance;

        int end_of_stance = getNextLiftoffIndex(contact_schedule, i, j);

        // Extend body motion if the upcoming stance runs past the horizon.
        Eigen::MatrixXd body_plan_stance;
        if (isContact(contact_schedule, end_of_stance, j)) {
          end_of_stance =
              std::max(static_cast<int>(i) + int(period_ * duty_cycles_[j]),
                       end_of_stance);

          body_plan_stance = Eigen::MatrixXd(end_of_stance + 1, 12);
          body_plan_stance.topRows(horizon_length_) = body_plan;
          for (size_t k = horizon_length_; k < end_of_stance; k++) {
            body_plan_stance.row(k) = computeFutureBodyPlan(
                k - (horizon_length_ - 1), body_plan.row(horizon_length_ - 1));
          }
        } else {
          body_plan_stance = body_plan;
        }

        // Center the foothold under the hip path during the stance window.
        std::vector<Eigen::Vector2d> P, R;
        for (size_t k = i; k < end_of_stance; k++) {
          quadKD_->worldToNominalHipFKWorldFrame(
              j, body_plan_stance.row(k).segment(0, 3),
              body_plan_stance.row(k).segment(3, 3), hip_position_midstance);

          Eigen::Vector2d p;
          p << hip_position_midstance.x(), hip_position_midstance.y();

          bool duplicate = false;
          for (size_t l = 0; l < P.size(); l++) {
            if ((P.at(l) - p).norm() < 1e-3) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) {
            P.push_back(p);
          }
        }

        hip_position_midstance = welzlMinimumCircle(P, R);

        body_vel_touchdown = body_plan.block<1, 3>(i, 6);
        ref_body_vel_touchdown = ref_body_plan.block<1, 3>(i, 6);
        body_ang_vel_touchdown = body_plan.block<1, 3>(i, 9);
        ref_body_ang_vel_touchdown = ref_body_plan.block<1, 3>(i, 9);

        double body_height_touchdown =
            std::max(body_plan(i, 2) -
                         terrain_grid_.atPosition(
                             "z_inpainted", body_plan.row(i).segment<2>(0)),
                     0.0);
        // Dynamic offsets combine centrifugal compensation and capture point.
        centrifugal = body_height_touchdown / 9.81 *
                      body_vel_touchdown.cross(ref_body_ang_vel_touchdown);
        vel_tracking = std::sqrt(body_height_touchdown / 9.81) *
                       (body_vel_touchdown - ref_body_vel_touchdown);

        // foot_position_grf =
        //     terrain_.projectToMap(hip_position_midstance, -1.0 *
        //     grf_midstance);

        foot_position_raibert =
            hip_position_midstance + centrifugal + vel_tracking;
        foot_position_nominal = foot_position_raibert;
        grid_map::Position foot_position_grid_map = {foot_position_nominal.x(),
                                                     foot_position_nominal.y()};

        if (!terrain_grid_.isInside(foot_position_grid_map)) {
          ++diag_outside;
          // Phase 2A: the bare `continue` below leaves this touchdown without a
          // fresh foothold; flag it so computeLocalPlan() can withhold the plan.
          record_foothold_failure(FootholdStatus::NOMINAL_OUTSIDE_MAP, j,
                                  static_cast<int>(i));
          RCLCPP_WARN(node_->get_logger(),
                      "Foot position is outside the map. Steer the robot in "
                      "another direction");
          continue;
        }
        ++diag_snap_calls;
        // Raise foothold by toe radius so the toe surface touches terrain.
        foot_position_nominal.z() =
            terrain_grid_.atPosition(
                "z_inpainted",
                terrain_grid_.getClosestPositionInMap(foot_position_grid_map),
                grid_map::InterpolationMethods::INTER_NEAREST) +
            toe_radius_;

        Eigen::Vector3d foot_position_previous =
            foot_positions.block<1, 3>(i, 3 * j);
        const FootholdResult foothold = getNearestValidFootholdResult(
            foot_position_nominal, foot_position_previous, j,
            hip_position_midstance);

        if (foothold.status == FootholdStatus::VALID) {
          foot_positions.block<1, 3>(i, 3 * j) = foothold.position;
        } else {
          // Phase 2A: no traversable / finite cell was found. Do NOT write the
          // hole/NaN nominal into the plan; inherit the previous touchdown value
          // (same as the non-touchdown branch) and flag the failure.
          record_foothold_failure(foothold.status, j, static_cast<int>(i));
          foot_positions.block<1, 3>(i, 3 * j) =
              getFootData(foot_positions, i - 1, j);
        }

        // [MPC_DOG Step 09] record what was selected + the raw-layer values at
        // the selected cell (answers: hole misread as safe, or snapped away).
        if (s09_dir != nullptr) {
          const double sx = foothold.position.x(), sy = foothold.position.y();
          const double s_zraw = step09Sample(terrain_grid_, "z", sx, sy);
          const double s_zinp = step09Sample(terrain_grid_, "z_inpainted", sx,
                                             sy);
          const double s_trav =
              step09Sample(terrain_grid_, obj_fun_layer_.c_str(), sx, sy);
          double s_hm = std::numeric_limits<double>::quiet_NaN();
          if (std::isfinite(s_zraw) && std::isfinite(s_zinp)) {
            s_hm = std::max(0.0, std::min(1.0, 1.0 - std::abs(s_zraw - s_zinp)));
          }
          const int s_obs = std::isfinite(s_zraw) ? 1 : 0;
          const int s_safe =
              (std::isfinite(s_trav) && s_trav > foothold_obj_threshold_) ? 1 : 0;
          char rb[768];
          // 20 columns, matching the header written at flush time.
          std::snprintf(
              rb, sizeof(rb),
              "%.4f,%ld,%s,%s,%zu,%.4f,"      // time,stamp,frame,leg,td_idx,td_time
              "%.5f,%.5f,%.5f,"               // nominal_x, nominal_y, nominal_trav
              "%.5f,%.5f,%.5f,"               // selected_x, selected_y, selected_z
              "%.5f,%.5f,%.5f,"               // sel_z_raw, sel_z_inpainted, sel_hole_mask
              "%d,%d,"                        // sel_observed, sel_binary_safe
              "%.5f,%.5f,%d",                 // snap_distance, hip_distance, status
              node_->now().seconds(),
              static_cast<long>(terrain_grid_.getTimestamp()),
              terrain_grid_.getFrameId().c_str(), kS09Legs[j], i,
              (current_plan_index + static_cast<int>(i)) * dt_,
              foot_position_nominal.x(), foot_position_nominal.y(),
              foothold.traversability_nominal, foothold.position.x(),
              foothold.position.y(), foothold.position.z(), s_zraw, s_zinp, s_hm,
              s_obs, s_safe, foothold.snap_distance,
              (foothold.position - hip_position_midstance).norm(),
              static_cast<int>(foothold.status));
          s09_foot_rows.emplace_back(rb);
        }

        // [MPC_DOG Step 11] once per leg per plan cycle (the first touchdown of
        // that leg in the horizon): enumerate reachable + safe + observed map
        // cells around the leg's hip at this touchdown pose, and record whether
        // the selected foothold passes the same tests. Shadow only.
        if (s09_dir != nullptr && !s11_leg_done[j] &&
            static_cast<int>(i) < body_plan.rows()) {
          s11_leg_done[j] = true;
          Eigen::Vector3d s11_hip;
          quadKD_->worldToNominalHipFKWorldFrame(
              j, body_plan.row(i).segment(0, 3),
              body_plan.row(i).segment(3, 3), s11_hip);
          step11EnumerateCandidates(
              terrain_grid_, obj_fun_layer_.c_str(), foothold_obj_threshold_,
              toe_radius_, s11_hip, ik_max_reach_, node_->now().seconds(),
              current_plan_index, kS09Legs[j], i, foothold.position.x(),
              foothold.position.y(), std::string(s09_dir));
        }

      } else {
        // Non-touchdown samples inherit the previous foothold.
        foot_positions.block<1, 3>(i, 3 * j) =
            getFootData(foot_positions, i - 1, j);
      }
    }
  }

  // Interpolate swing trajectories between past and planned footholds.
  for (int j = 0; j < num_feet_; j++) {
    quad_msgs::msg::FootState most_recent_foothold_msg =
        past_footholds_msg.feet[j];

    int i_liftoff = most_recent_foothold_msg.traj_index - current_plan_index;
    int i_touchdown = getNextContactIndex(contact_schedule, 0, j);
    double swing_duration = i_touchdown - i_liftoff;

    Eigen::Vector3d foot_position_prev, foot_position_prev_nominal;
    quad_utils::footStateMsgToEigen(most_recent_foothold_msg,
                                    foot_position_prev);
    quad_utils::footStateMsgToEigen(most_recent_foothold_msg,
                                    foot_position_prev_nominal);
    Eigen::Vector3d foot_velocity_prev;
    foot_velocity_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d foot_position_next =
        getFootData(foot_positions, i_touchdown, j);
    Eigen::Vector3d foot_velocity_next = Eigen::Vector3d::Zero();

    int mid_air = std::round(i_liftoff + swing_duration / 2);

    Eigen::VectorXd body_plan_mid_air;
    double swing_apex;

    if (mid_air < 0) {
      // Reuse the stored apex when the swing started before this horizon.
      swing_apex = most_recent_foothold_msg.velocity.z;
    } else {
      body_plan_mid_air = body_plan.row(mid_air);

      swing_apex = computeSwingApex(
          j, body_plan_mid_air, foot_position_prev_nominal, foot_position_next);

      // Store apex in velocity.z; this message field is otherwise unused here.
      past_footholds_msg.feet[j].velocity.z = swing_apex;
    }

    for (int i = 0; i < contact_schedule.size(); i++) {
      quad_msgs::msg::FootState foot_state_msg;
      foot_state_msg.traj_index = current_plan_index + i;

      Eigen::Vector3d foot_position;
      Eigen::Vector3d foot_velocity;
      Eigen::Vector3d foot_acceleration;

      if (!isContact(contact_schedule, i, j) ||
          (i != 0 && isNewContact(contact_schedule, i, j))) {
        // New contact still uses swing acceleration for inverse dynamics.
        if (isNewLiftoff(contact_schedule, i, j)) {
          i_liftoff = i;
          foot_position_prev = getFootData(foot_positions, i_liftoff, j);
          foot_position_prev_nominal =
              getFootData(foot_positions, i_liftoff, j);
          foot_velocity_prev = Eigen::Vector3d::Zero();

          i_touchdown = getNextContactIndex(contact_schedule, i, j);

          // Predict touchdown heuristically when it falls beyond this horizon.
          if (!isContact(contact_schedule, i_touchdown, j)) {
            int stance_duration = period_ * (duty_cycles_[j]);
            swing_duration = period_ - stance_duration;

            Eigen::VectorXd body_plan_midstance = computeFutureBodyPlan(
                (i_liftoff + swing_duration + 0.5 * stance_duration) -
                    (horizon_length_ - 1),
                body_plan.row(horizon_length_ - 1));
            Eigen::Vector3d body_pos_midstance =
                body_plan_midstance.segment(0, 3);
            Eigen::Vector3d body_rpy_midstance =
                body_plan_midstance.segment(3, 3);
            quadKD_->worldToNominalHipFKWorldFrame(
                j, body_pos_midstance, body_rpy_midstance, foot_position_next);

            grid_map::Position foot_position_next_grid_map =
                foot_position_next.head(2);
            if (!terrain_grid_.isInside(foot_position_next_grid_map)) {
              RCLCPP_WARN(
                  node_->get_logger(),
                  "computeFootPlan prediction receives a position out of "
                  "range, pick the previous position in map!");
              foot_position_next_grid_map = foot_position_prev.head(2);
              foot_position_next = foot_position_prev.head(2);
            }
            foot_position_next.z() =
                terrain_grid_.atPosition(
                    "z_inpainted", foot_position_next_grid_map,
                    grid_map::InterpolationMethods::INTER_NEAREST) +
                toe_radius_;
          } else {
            foot_position_next = getFootData(foot_positions, i_touchdown, j);
            swing_duration = i_touchdown - i_liftoff;
          }

          mid_air = std::round(i_liftoff + swing_duration / 2);
          if (mid_air > horizon_length_ - 1) {
            body_plan_mid_air = computeFutureBodyPlan(
                (i_liftoff + swing_duration / 2) - (horizon_length_ - 1),
                body_plan.row(horizon_length_ - 1));
          } else {
            body_plan_mid_air = body_plan.row(mid_air);
          }

          swing_apex =
              computeSwingApex(j, body_plan_mid_air, foot_position_prev_nominal,
                               foot_position_next);
        }

        // First sample may be partway through a timestep; adjust swing phase.
        double swing_idx =
            (i == 0) ? (i - i_liftoff + (dt_ - first_element_duration) / dt_)
                     : (i - i_liftoff);
        double swing_idx_current =
            0 - i_liftoff + (dt_ - first_element_duration) / dt_;

        double interp_phase;
        double interp_duration;

        interp_phase = swing_idx / swing_duration;
        interp_duration = swing_duration * dt_;

        // Interpolate horizontal swing with endpoint velocities fixed.
        cubicHermiteSpline(foot_position_prev.x(), foot_velocity_prev.x(),
                           foot_position_next.x(), foot_velocity_next.x(),
                           interp_phase, interp_duration, foot_position.x(),
                           foot_velocity.x(), foot_acceleration.x());
        cubicHermiteSpline(foot_position_prev.y(), foot_velocity_prev.y(),
                           foot_position_next.y(), foot_velocity_next.y(),
                           interp_phase, interp_duration, foot_position.y(),
                           foot_velocity.y(), foot_acceleration.y());

        // Split vertical swing into up/down splines through the apex.
        interp_phase = (2 * swing_idx >= swing_duration)
                           ? (2 * swing_idx / swing_duration - 1)
                           : (2 * swing_idx / swing_duration);
        interp_duration = swing_duration * dt_ / 2;

        if (swing_idx / swing_duration < 0.5) {
          cubicHermiteSpline(foot_position_prev.z(), foot_velocity_prev.z(),
                             swing_apex, 0, interp_phase, interp_duration,
                             foot_position.z(), foot_velocity.z(),
                             foot_acceleration.z());
        } else {
          cubicHermiteSpline(swing_apex, 0, foot_position_next.z(),
                             foot_velocity_next.z(), interp_phase,
                             interp_duration, foot_position.z(),
                             foot_velocity.z(), foot_acceleration.z());
        }

        foot_state_msg.contact = false;
      }

      if (isContact(contact_schedule, i, j)) {
        foot_position = getFootData(foot_positions, i, j);
        foot_velocity = Eigen::VectorXd::Zero(3);
        if (!(i != 0 && isNewContact(contact_schedule, i, j))) {
          // Preserve touchdown acceleration only at the contact transition.
          foot_acceleration = Eigen::VectorXd::Zero(3);
        }

        foot_state_msg.contact = true;
      }

      quad_utils::eigenToFootStateMsg(foot_position, foot_velocity,
                                      foot_acceleration, foot_state_msg);
      foot_positions.block<1, 3>(i, 3 * j) = foot_position;
      foot_velocities.block<1, 3>(i, 3 * j) = foot_velocity;
      foot_accelerations.block<1, 3>(i, 3 * j) = foot_acceleration;

      // Cache liftoff state for the next horizon's swing interpolation.
      if (i < period_ * duty_cycles_[j] &&
          isNewLiftoff(contact_schedule, i, j)) {
        past_footholds_msg.feet[j] = foot_state_msg;
      }
    }
  }
  static long diag_cfp_calls = 0;
  if (diag_cfp_calls++ % 100 == 0) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[DIAG] computeFootPlan #%ld: new_contacts=%d snap_calls=%d outside=%d "
        "plan_index=%d cs_len=%zu",
        diag_cfp_calls, diag_new_contacts, diag_snap_calls, diag_outside,
        current_plan_index, contact_schedule.size());
  }

  // [MPC_DOG Step 09] measurement-only: flush the per-touchdown rows (append)
  // and, once, the terrain-map cross-section. No effect on plan_result.
  if (s09_dir != nullptr) {
    const std::string foot_path = std::string(s09_dir) + "/step09_footholds.csv";
    const bool need_header = !std::ifstream(foot_path).good();
    std::ofstream ff(foot_path, std::ios::app);
    if (ff) {
      if (need_header) {
        ff << "time,map_stamp,frame,leg,touchdown_index,touchdown_time,"
              "nominal_x,nominal_y,nominal_traversability,selected_x,selected_y,"
              "selected_z,selected_z_raw,selected_z_inpainted,"
              "selected_hole_mask_recon,selected_observed,selected_binary_safe,"
              "snap_distance,hip_distance,foothold_status\n";
      }
      for (const auto &r : s09_foot_rows) ff << r << "\n";
    }
    static bool s09_map_done = false;
    if (!s09_map_done && terrain_grid_.getSize()(0) > 0 &&
        terrain_grid_.exists("traversability")) {
      step09DumpMapCrossSection(terrain_grid_, foothold_obj_threshold_, s09_dir);
      s09_map_done = true;
    }

    // [MPC_DOG Step 10] flush the future gait-event rows (append).
    const std::string gait_path =
        std::string(s09_dir) + "/step10_gait_events.csv";
    const bool gait_hdr = !std::ifstream(gait_path).good();
    std::ofstream gf(gait_path, std::ios::app);
    if (gf) {
      if (gait_hdr) {
        gf << "time,current_plan_index,phase,period,dt,leg,event_ordinal,"
              "pred_touchdown_horizon_index,pred_touchdown_time\n";
      }
      for (const auto &r : s10_gait_rows) gf << r << "\n";
    }
  }

  return plan_result;
}

void LocalFootstepPlanner::loadFootPlanMsgs(
    const std::vector<std::vector<bool>>& contact_schedule,
    int current_plan_index, double first_element_duration,
    const Eigen::MatrixXd& foot_positions,
    const Eigen::MatrixXd& foot_velocities,
    const Eigen::MatrixXd& foot_accelerations,
    quad_msgs::msg::MultiFootPlanDiscrete& future_footholds_msg,
    quad_msgs::msg::MultiFootPlanContinuous& foot_plan_continuous_msg) {
  foot_plan_continuous_msg.states.resize(contact_schedule.size());
  future_footholds_msg.feet.resize(num_feet_);

  // Build continuous trajectory samples and discrete touchdown messages.
  for (int j = 0; j < num_feet_; j++) {
    future_footholds_msg.feet[j].header = future_footholds_msg.header;

    for (int i = 0; i < contact_schedule.size(); i++) {
      // Stamp each horizon sample once, independent of foot index.
      if (j == 0) {
        foot_plan_continuous_msg.states[i].header =
            foot_plan_continuous_msg.header;
        if (i == 0) {
          foot_plan_continuous_msg.states[i].header.stamp =
              foot_plan_continuous_msg.header.stamp;
        } else {
          foot_plan_continuous_msg.states[i].header.stamp =
              foot_plan_continuous_msg.header.stamp +
              rclcpp::Duration::from_seconds(first_element_duration) +
              rclcpp::Duration::from_seconds((i - 1) * dt_);
        }
        foot_plan_continuous_msg.states[i].traj_index = current_plan_index + i;
      }

      quad_msgs::msg::FootState foot_state_msg;
      foot_state_msg.header = foot_plan_continuous_msg.header;
      foot_state_msg.traj_index = foot_plan_continuous_msg.states[i].traj_index;
      quad_utils::eigenToFootStateMsg(foot_positions.block<1, 3>(i, 3 * j),
                                      foot_velocities.block<1, 3>(i, 3 * j),
                                      foot_accelerations.block<1, 3>(i, 3 * j),
                                      foot_state_msg);
      foot_state_msg.contact = isContact(contact_schedule, i, j);

      if (isNewContact(contact_schedule, i, j)) {
        future_footholds_msg.feet[j].footholds.push_back(foot_state_msg);
      }

      foot_plan_continuous_msg.states[i].feet.push_back(foot_state_msg);
    }
  }
}

Eigen::Vector3d LocalFootstepPlanner::getNearestValidFoothold(
    const Eigen::Vector3d& foot_position,
    const Eigen::Vector3d& foot_position_prev_solve) const {
  // Wrapper: existing callers only need the position, and it is identical to
  // what this function returned before the FootholdResult refactor.
  return getNearestValidFootholdResult(foot_position, foot_position_prev_solve)
      .position;
}

FootholdResult LocalFootstepPlanner::getNearestValidFootholdResult(
    const Eigen::Vector3d& foot_position,
    const Eigen::Vector3d& foot_position_prev_solve, int leg_index,
    const Eigen::Vector3d& hip_world) const {
  FootholdResult result;
  // Default = nominal, matching the prior "return nominal" fallback path.
  result.position = foot_position;

  // Defensive: nominal outside the map. In the current call flow
  // computeFootPlan() already `continue`s before reaching here, so this
  // branch is inert today; it exists so the status can be reported once a
  // later phase moves/adds the out-of-map handling.
  if (!terrain_grid_.isInside(foot_position.head<2>())) {
    result.status = FootholdStatus::NOMINAL_OUTSIDE_MAP;
    return result;
  }
  result.traversability_nominal =
      terrain_grid_.atPosition(obj_fun_layer_, foot_position.head<2>());

  Eigen::Vector3d foot_position_best = foot_position;
  grid_map::Position pos_center, pos_center_aligned, offset, pos_valid;

  // Preserve sub-cell offset while scanning nearby grid cells.
  pos_center = foot_position.head<2>();
  grid_map::Index idx;
  terrain_grid_.getIndex(pos_center, idx);
  terrain_grid_.getPosition(idx, pos_center_aligned);
  offset = pos_center - pos_center_aligned;
  double best_kin_cost = std::numeric_limits<double>::max();

  // Spiral outward from the nominal foothold and keep the best valid cell.
  for (grid_map::SpiralIterator iterator(terrain_grid_, pos_center_aligned,
                                         foothold_search_radius_);
       !iterator.isPastEnd(); ++iterator) {
    terrain_grid_.getPosition(*iterator, pos_valid);
    pos_valid += offset;

    if (!terrain_grid_.isInside(pos_valid)) {
      continue;
    }

    double traversability = terrain_grid_.atPosition(obj_fun_layer_, pos_valid);
    double kin_cost =
        (pos_valid - foot_position.head<2>()).norm() +
        0.5 * (pos_valid - foot_position_prev_solve.head<2>()).norm();
    // [MPC_DOG] Stock kin_cost (dist-to-nominal + 0.5*dist-to-prev-solve).
    // Earlier rounds added a huge penalty here to FORBID snapping toward the
    // near edge of a hole, forcing a single commit-step across. That made the
    // footstep planner plant a front foot on the FAR edge while the body was
    // still ~0.5 m short of it -> the stance foot fell outside the leg's
    // kinematic reach -> the centroidal NMPC cost exploded and the body
    // lunged/pitched into the hole. Letting the natural cost pick the nearest
    // valid cell instead makes the robot STAGE the crossing: it steps to the
    // near edge first (prev-solve foot is behind, so the near edge wins),
    // shifts its body up, then the next step lands on the far strip as an
    // ordinary un-snapped foothold once it is within reach.

    if (traversability > foothold_obj_threshold_ &&
        (kin_cost < best_kin_cost)) {
      foot_position_best.head<2>() = pos_valid;
      best_kin_cost = kin_cost;
    }
  }

  const bool found = best_kin_cost != std::numeric_limits<double>::max();
  if (!found) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(),
        static_cast<rcutils_duration_value_t>(1e9),
        "No valid foothold found in radius of nominal, returning nominal");
    result.status = FootholdStatus::NO_TRAVERSABLE_CANDIDATE;
  }

  // Height query is unchanged: on the no-candidate path foot_position_best.xy
  // is still the nominal, so this reproduces the prior return value exactly.
  foot_position_best.z() =
      terrain_grid_.atPosition("z_inpainted", foot_position_best.head<2>(),
                               grid_map::InterpolationMethods::INTER_LINEAR) +
      toe_radius_;
  result.position = foot_position_best;

  if (found) {
    if (!std::isfinite(foot_position_best.z())) {
      result.status = FootholdStatus::NONFINITE_HEIGHT;
    } else {
      result.status = FootholdStatus::VALID;
      result.traversability_selected =
          terrain_grid_.atPosition(obj_fun_layer_, foot_position_best.head<2>());
      result.snap_distance =
          (foot_position_best.head<2>() - foot_position.head<2>()).norm();
    }
  }

  // Phase 3: forward probe from the chosen foothold along the travel direction
  // (+x for the transverse full-width gaps in these scenarios and Step 05). If a
  // hole starts within edge_clearance_ (the foothold is on a lip) AND solid
  // ground does not resume within max_crossable_gap_ past the hole start (no
  // reachable far side), the foothold is EDGE_TOO_CLOSE and Phase 2A withholds
  // the plan. A lip before a *crossable* gap (step03/04's 0.3 m trench) stays
  // VALID. A hole further ahead than edge_clearance_, or one behind the
  // foothold, does not matter. Reaching the edge of the mapped area stops the
  // probe WITHOUT flagging: the map boundary is unmapped ground, not a cliff
  // (this removed spurious EDGE_TOO_CLOSE on far-horizon footholds that landed
  // near the last mapped strip in Step 05). edge_clearance_ == 0 disables the
  // check (pre-Phase-3 behaviour); max_crossable_gap_ == 0 makes any lip
  // EDGE_TOO_CLOSE.
  if (result.status == FootholdStatus::VALID && edge_clearance_ > 0.0) {
    const Eigen::Vector2d fwd(1.0, 0.0);
    const double step = std::max(terrain_grid_.getResolution(), 1e-3);
    const double max_d = edge_clearance_ + std::max(max_crossable_gap_, 0.0);
    bool on_lip = false;
    double hole_start = 0.0;
    for (double d = step; d <= max_d; d += step) {
      const grid_map::Position p = foot_position_best.head<2>() + d * fwd;
      if (!terrain_grid_.isInside(p)) {
        break;  // edge of the mapped area -> unknown, not a cliff
      }
      const double t = terrain_grid_.atPosition(obj_fun_layer_, p);
      const bool unsafe = !std::isfinite(t) || t <= foothold_obj_threshold_;
      if (!on_lip) {
        if (unsafe) {
          if (d > edge_clearance_) break;  // hole too far ahead -> foothold ok
          on_lip = true;
          hole_start = d;
          result.edge_clearance = d;
        }
        continue;
      }
      if (!unsafe) break;  // far side within reach -> crossable, stays VALID
      if (d - hole_start >= max_crossable_gap_) {
        result.status = FootholdStatus::EDGE_TOO_CLOSE;  // no far side in reach
        break;
      }
    }
  }

  // Phase 4: is the chosen foothold within this leg's reach? The footstep
  // search can snap a foothold up to foothold_search_radius (0.7 m) away, past
  // the leg's ~0.42 m full extension, and NMPC does not check reach either ->
  // an unreachable foothold becomes a fixed NMPC parameter and the stance
  // degenerates. Geometric check against the midstance hip this foot supports
  // (passed in from computeFootPlan): a raw IK "not exact" flag was too noisy
  // (it trips on every small clamp during normal walking). ik_reach_check_ == 0
  // disables this.
  if (result.status == FootholdStatus::VALID && ik_reach_check_ &&
      leg_index >= 0) {
    if ((result.position - hip_world).norm() > ik_max_reach_) {
      result.status = FootholdStatus::IK_UNREACHABLE;
    }
  }

  // [MPC_DOG DIAG] nominal vs snapped foothold + status + snap distance
  {
    static long diag_gnvf = 0;
    if (diag_gnvf++ % 40 == 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "[DIAG] gnvf #%ld: nominal x=%.3f trav=%.3f -> snapped "
                  "x=%.3f (thr=%.2f rad=%.2f) found=%d status=%d snap=%.3f "
                  "edge_clr=%.3f",
                  diag_gnvf, foot_position.x(), result.traversability_nominal,
                  result.position.x(), foothold_obj_threshold_,
                  foothold_search_radius_, found,
                  static_cast<int>(result.status), result.snap_distance,
                  result.edge_clearance);
    }
  }

  return result;
}

bool LocalFootstepPlanner::hasUncrossableGapAhead(const Eigen::Vector2d& from,
                                                  double lookahead) const {
  if (edge_clearance_ <= 0.0 || max_crossable_gap_ <= 0.0 || lookahead <= 0.0) {
    return false;  // Phase 3 opt-in only
  }
  const Eigen::Vector2d fwd(1.0, 0.0);
  const double step = std::max(terrain_grid_.getResolution(), 1e-3);
  bool in_hole = false;
  double hole_start = 0.0;
  for (double d = step; d <= lookahead; d += step) {
    const grid_map::Position p = from + d * fwd;
    if (!terrain_grid_.isInside(p)) {
      return false;  // beyond the mapped area -> unknown, not a cliff
    }
    const double t = terrain_grid_.atPosition(obj_fun_layer_, p);
    const bool unsafe = !std::isfinite(t) || t <= foothold_obj_threshold_;
    if (unsafe && !in_hole) {
      in_hole = true;
      hole_start = d;
    } else if (!unsafe && in_hole) {
      in_hole = false;  // a strip resumed -> that gap was crossable
    }
    if (in_hole && d - hole_start >= max_crossable_gap_) {
      return true;  // hole with no far side within reach
    }
  }
  return false;
}

Eigen::Vector3d LocalFootstepPlanner::welzlMinimumCircle(
    std::vector<Eigen::Vector2d> P, std::vector<Eigen::Vector2d> R) {
  if (R.size() == 3) {
    // Base case: circle through three boundary points.
    Eigen::Vector3d D;
    D << (0.5000 * (R.at(0).x() * R.at(0).x() * R.at(1).y() -
                    R.at(0).x() * R.at(0).x() * R.at(2).y() -
                    R.at(1).x() * R.at(1).x() * R.at(0).y() +
                    R.at(1).x() * R.at(1).x() * R.at(2).y() +
                    R.at(2).x() * R.at(2).x() * R.at(0).y() -
                    R.at(2).x() * R.at(2).x() * R.at(1).y() +
                    R.at(0).y() * R.at(0).y() * R.at(1).y() -
                    R.at(0).y() * R.at(0).y() * R.at(2).y() -
                    R.at(0).y() * R.at(1).y() * R.at(1).y() +
                    R.at(0).y() * R.at(2).y() * R.at(2).y() +
                    R.at(1).y() * R.at(1).y() * R.at(2).y() -
                    R.at(1).y() * R.at(2).y() * R.at(2).y())) /
             (R.at(0).x() * R.at(1).y() - R.at(1).x() * R.at(0).y() -
              R.at(0).x() * R.at(2).y() + R.at(2).x() * R.at(0).y() +
              R.at(1).x() * R.at(2).y() - R.at(2).x() * R.at(1).y()),
        (0.5000 * (-R.at(0).x() * R.at(0).x() * R.at(1).x() +
                   R.at(0).x() * R.at(0).x() * R.at(2).x() +
                   R.at(0).x() * R.at(1).x() * R.at(1).x() -
                   R.at(0).x() * R.at(2).x() * R.at(2).x() +
                   R.at(0).x() * R.at(1).y() * R.at(1).y() -
                   R.at(0).x() * R.at(2).y() * R.at(2).y() -
                   R.at(1).x() * R.at(1).x() * R.at(2).x() +
                   R.at(1).x() * R.at(2).x() * R.at(2).x() -
                   R.at(1).x() * R.at(0).y() * R.at(0).y() +
                   R.at(1).x() * R.at(2).y() * R.at(2).y() +
                   R.at(2).x() * R.at(0).y() * R.at(0).y() -
                   R.at(2).x() * R.at(1).y() * R.at(1).y())) /
            (R.at(0).x() * R.at(1).y() - R.at(1).x() * R.at(0).y() -
             R.at(0).x() * R.at(2).y() + R.at(2).x() * R.at(0).y() +
             R.at(1).x() * R.at(2).y() - R.at(2).x() * R.at(1).y()),
        (0.5000 *
         std::sqrt((R.at(0).x() * R.at(0).x() - 2 * R.at(0).x() * R.at(1).x() +
                    R.at(1).x() * R.at(1).x() + R.at(0).y() * R.at(0).y() -
                    2 * R.at(0).y() * R.at(1).y() + R.at(1).y() * R.at(1).y()) *
                   (R.at(0).x() * R.at(0).x() - 2 * R.at(0).x() * R.at(2).x() +
                    R.at(2).x() * R.at(2).x() + R.at(0).y() * R.at(0).y() -
                    2 * R.at(0).y() * R.at(2).y() + R.at(2).y() * R.at(2).y()) *
                   (R.at(1).x() * R.at(1).x() - 2 * R.at(1).x() * R.at(2).x() +
                    R.at(2).x() * R.at(2).x() + R.at(1).y() * R.at(1).y() -
                    2 * R.at(1).y() * R.at(2).y() +
                    R.at(2).y() * R.at(2).y()))) /
            std::abs(R.at(0).x() * R.at(1).y() - R.at(1).x() * R.at(0).y() -
                     R.at(0).x() * R.at(2).y() + R.at(2).x() * R.at(0).y() +
                     R.at(1).x() * R.at(2).y() - R.at(2).x() * R.at(1).y());

    return D;
  }

  if (P.empty()) {
    // Base cases for zero, one, or two boundary points.
    Eigen::Vector3d D;
    switch (R.size()) {
      case 0:
        D << 0, 0, 0;
        break;
      case 1:
        D << R.at(0).x(), R.at(0).y(), 0;
        break;
      case 2:
        D << (R.at(0).x() + R.at(1).x()) / 2, (R.at(0).y() + R.at(1).y()) / 2,
            (R.at(0) - R.at(1)).norm() / 2;
        break;

      default:
        D << 0, 0, 0;
        break;
    }

    return D;
  }

  Eigen::Vector2d p = P.back();
  P.pop_back();
  Eigen::Vector3d D = welzlMinimumCircle(P, R);
  if ((p - D.segment(0, 2)).norm() < D.z()) {
    return D;
  }

  R.push_back(p);
  return welzlMinimumCircle(P, R);
}

double LocalFootstepPlanner::computeSwingApex(
    int leg_idx, const Eigen::VectorXd& body_plan,
    const Eigen::Vector3d& foot_position_prev,
    const Eigen::Vector3d& foot_position_next) {
  Eigen::Matrix4d g_world_legbase;
  quadKD_->worldToLegbaseFKWorldFrame(leg_idx, body_plan.segment(0, 3),
                                      body_plan.segment(3, 3), g_world_legbase);
  double hip_height = g_world_legbase(2, 3);
  double max_extension = 0.35;

  // Keep swing high enough for terrain, but below hip clearance limits.
  double swing_apex =
      std::min(ground_clearance_ - toe_radius_ +
                   std::max(foot_position_prev.z(), foot_position_next.z()),
               hip_height - hip_clearance_);
  swing_apex = std::max(swing_apex, hip_height - max_extension);

  return swing_apex;
}
