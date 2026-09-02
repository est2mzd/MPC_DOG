#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <tf2_eigen/tf2_eigen.hpp>
#include <local_planner/local_planner_modes.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quad_msgs/msg/foot_plan_discrete.hpp>
#include <quad_msgs/msg/foot_state.hpp>
#include <quad_msgs/msg/multi_foot_plan_continuous.hpp>
#include <quad_msgs/msg/multi_foot_plan_discrete.hpp>
#include <quad_msgs/msg/multi_foot_state.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/fast_terrain_map.hpp>
#include <quad_utils/function_timer.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/primitive_ids.hpp>
#include <quad_utils/quad_kd2.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Eigen>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <limits>

//! Outcome of a single foothold-selection query.
/*!
   Phase 1 (diagnostics only): the four values below are the states
   getNearestValidFootholdResult() can currently report. EDGE_TOO_CLOSE /
   IK_UNREACHABLE / MAP_STALE are intentionally NOT here yet; they are added
   in later phases together with the code that actually computes them, so the
   cause and effect of each change stay easy to follow.
*/
enum class FootholdStatus {
  VALID,                     //!< A traversable candidate was selected
  NOMINAL_OUTSIDE_MAP,       //!< Nominal foothold lies outside the terrain map
  NO_TRAVERSABLE_CANDIDATE,  //!< No cell in the search radius passed the threshold
  NONFINITE_HEIGHT,          //!< Selected cell's inpainted height is not finite
  EDGE_TOO_CLOSE,  //!< Selected cell is within edge_clearance of a hole/off-map cell
  IK_UNREACHABLE,  //!< Selected cell cannot be reached by the leg's inverse kinematics
};

//! Result of getNearestValidFootholdResult().
/*!
   Phase 1 fills only fields that are cheap to compute and do not change any
   selection behaviour: the chosen position (identical to what
   getNearestValidFoothold returns), the status, the objective-layer value at
   the nominal and at the chosen cell, and the horizontal snap distance.
   edge clearance and IK reachability are added in later phases.
*/
struct FootholdResult {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  //!< Chosen foothold, world
  FootholdStatus status = FootholdStatus::VALID;
  double traversability_nominal =
      std::numeric_limits<double>::quiet_NaN();  //!< obj-layer value at nominal
  double traversability_selected =
      std::numeric_limits<double>::quiet_NaN();  //!< obj-layer value at chosen
  double snap_distance = 0.0;  //!< ||chosen.xy - nominal.xy||, metres
  double edge_clearance =
      std::numeric_limits<double>::quiet_NaN();  //!< Phase 3: distance from the
                                                 //!< chosen cell to the nearest
                                                 //!< unsafe cell (clamped to the
                                                 //!< scan radius); NaN if the
                                                 //!< check is disabled
};

//! Aggregate outcome of one computeFootPlan() call over the whole horizon.
/*!
   Phase 2A: computeFootPlan() walks every touchdown event of every leg. If any
   touchdown could not be placed on a traversable cell (status != VALID, or the
   nominal fell outside the map) the plan is not safe to hand to NMPC. This
   struct reports that so LocalPlanner::computeLocalPlan() can withhold the
   local plan (see local_planner.stop_on_invalid_foothold). It carries the
   first failure's details plus a total count; it never changes the footholds.
*/
struct FootPlanResult {
  bool ok = true;                //!< false if any touchdown failed placement
  FootholdStatus worst_status =  //!< status of the first failing touchdown
      FootholdStatus::VALID;
  int failed_leg = -1;              //!< leg index of the first failure, -1 if none
  int failed_touchdown_index = -1;  //!< horizon index of the first failure
  int failed_count = 0;             //!< total failing touchdowns this call
  int nearest_failed_index = -1;    //!< smallest horizon index among failures
                                    //!< (-1 if none); Phase 2B uses this to
                                    //!< decide how soon to start the stop

  //! [MPC_DOG Step 12/14] multi-step foothold-sequence shadow result.
  int multistep_blocked_k = -1;     //!< touchdown step where the sequence blocks
  int multistep_blocked_leg = -1;   //!< leg index at that step
  bool multistep_stop_request = false;  //!< blocked within final_stop_steps
  bool multistep_slow = false;          //!< blocked, but beyond final_stop_steps
  int multistep_applied_footholds = 0;  //!< [Step 15] touchdowns whose nominal
                                        //!< was replaced by a planned foothold
                                        //!< this call (0 unless apply_foothold)
};

//! Local footstep planner for quadruped body plans
/*!
   LocalFootstepPlanner converts a body plan and contact schedule into
   discrete footholds and continuous swing-foot trajectories.
*/
class LocalFootstepPlanner {
 public:
  /**
   * @brief Constructor for LocalFootstepPlanner Class
   * @return Constructed object of type LocalFootstepPlanner
   */
  LocalFootstepPlanner(rclcpp::Node::SharedPtr node);

  /**
   * @brief Set the temporal parameters of this object
   * @param[in] dt The duration of one timestep in the plan
   * @param[in] period The period of a gait cycle in number of timesteps
   * @param[in] horizon_length The length of the planning horizon in number of
   * timesteps
   * @param[in] duty_cycles Fraction of gait period each foot stays in contact
   * @param[in] phase_offsets Phase offset for each foot touchdown
   */
  void setTemporalParams(double dt, int period, int horizon_length,
                         const std::vector<double>& duty_cycles,
                         const std::vector<double>& phase_offsets);

  /**
   * @brief Set the spatial parameters of this object
   * @param[in] ground_clearance The foot clearance over adjacent footholds in m
   * @param[in] hip_clearance The foot clearance under hip in m
   * @param[in] grf_weight Weight on GRF projection (0 to 1)
   * @param[in] standing_error_threshold Threshold of body error from desired
   * goal to start stepping
   * @param[in] kinematics Kinematics class for computations
   * @param[in] foothold_search_radius Radius to locally search for valid
   * footholds (m)
   * @param[in] foothold_obj_threshold Minimum objective function value for
   * valid foothold
   * @param[in] obj_fun_layer Terrain layer for foothold search
   * @param[in] toe_radius Toe radius
   */
  void setSpatialParams(double ground_clearance, double hip_clearance,
                        double grf_weight, double standing_error_threshold,
                        std::shared_ptr<quad_utils::QuadKD2> kinematics,
                        double foothold_search_radius,
                        double foothold_obj_threshold,
                        std::string obj_fun_layer, double toe_radius,
                        double edge_clearance = 0.0,
                        double max_crossable_gap = 0.6,
                        bool ik_reach_check = false,
                        double ik_max_reach = 0.45);

  /**
   * @brief [MPC_DOG Step 14] Configure the multi-step foothold-sequence shadow
   * planner. When `enabled`, the sequence search runs every 5th cycle even
   * without the CSV-dump env; when `apply_stop_request`, a BLOCKED_AT_STEP_K
   * within final_stop_steps sets FootPlanResult::multistep_stop_request so the
   * caller can latch the existing Phase 2B graceful stop. When `apply_foothold`
   * ([MPC_DOG Step 15]), each leg's nearest touchdown nominal is replaced by
   * the planned foothold (when one is available and sane) before the existing
   * getNearestValidFootholdResult snap runs as the final local correction.
   */
  void setMultistepParams(bool enabled, bool apply_stop_request,
                          bool apply_foothold, int stop_margin_steps,
                          double planning_distance);

  /**
   * @brief Transform a vector of foot positions from the world to the body
   * frame
   * @param[in] body_plan Current body plan
   * @param[in] foot_positions_world Foot positions in the world frame
   * @param[out] foot_positions_body Foot positions in the body frame
   */
  void getFootPositionsBodyFrame(const Eigen::VectorXd& body_plan,
                                 const Eigen::VectorXd& foot_positions_world,
                                 Eigen::VectorXd& foot_positions_body);

  /**
   * @brief Transform the entire foot plan from the world to the body frame
   * @param[in] body_plan Current body plan
   * @param[in] foot_positions_world Foot positions in the world frame
   * @param[out] foot_positions_body Foot positions in the body frame
   */
  void getFootPositionsBodyFrame(const Eigen::MatrixXd& body_plan,
                                 const Eigen::MatrixXd& foot_positions_world,
                                 Eigen::MatrixXd& foot_positions_body);

  /**
   * @brief Update the fast terrain map used for surface normal queries
   * @param[in] terrain Terrain map wrapper
   */
  void updateMap(const FastTerrainMap& terrain);

  /**
   * @brief Update the grid map used for height and traversability queries
   * @param[in] terrain Terrain grid map
   */
  void updateMap(const grid_map::GridMap& terrain);

  /**
   * @brief Compute the contact schedule based on the current phase
   * @param[in] current_plan_index Current index in the plan
   * @param[in] body_plan Current body plan
   * @param[in] ref_primitive_plan Reference primitive plan
   * @param[in] control_mode Control mode
   * @param[out] contact_schedule 2D array of contact states
   */
  void computeContactSchedule(int current_plan_index,
                              const Eigen::MatrixXd& body_plan,
                              const Eigen::VectorXi& ref_primitive_plan_,
                              int control_mode,
                              std::vector<std::vector<bool>>& contact_schedule);

  /**
   * @brief Update the discrete footstep plan with the current plan
   * @param[in] current_plan_index Current plan index
   * @param[in] contact_schedule Current contact schedule
   * @param[in] body_plan Current body plan
   * @param[in] grf_plan Current grf plan
   * @param[in] ref_body_plan Reference body plan
   * @param[in] foot_positions_current Current foot position in the world frame
   * @param[in] foot_velocities_current Current foot velocity in the world frame
   * @param[in] first_element_duration Duration of first element of horizon (may
   * not be dt)
   * @param[in] past_footholds_msg Message of past footholds, used for
   * interpolation of swing state
   * @param[out] foot_positions Foot positions over the horizon
   * @param[out] foot_velocities Foot velocities over the horizon
   * @param[out] foot_accelerations Foot accelerations over the horizon
   * @return FootPlanResult: ok=false (with first-failure details + count) if any
   * touchdown could not be placed on a traversable in-map cell. Footholds are
   * unchanged relative to the pre-Phase-2A behaviour except that a failed
   * touchdown now inherits the previous foothold instead of a hole/NaN cell.
   */
  FootPlanResult computeFootPlan(
      int current_plan_index,
      const std::vector<std::vector<bool>>& contact_schedule,
      const Eigen::MatrixXd& body_plan, const Eigen::MatrixXd& grf_plan,
      const Eigen::MatrixXd& ref_body_plan,
      const Eigen::VectorXd& foot_positions_current,
      const Eigen::VectorXd& foot_velocities_current,
      double first_element_duration,
      quad_msgs::msg::MultiFootState& past_footholds_msg,
      Eigen::MatrixXd& foot_positions, Eigen::MatrixXd& foot_velocities,
      Eigen::MatrixXd& foot_accelerations);

  /**
   * @brief Phase 2B: is there an uncrossable gap on the path ahead?
   *
   * Marches the terrain map forward (+x, the travel direction) from `from` up
   * to `lookahead` metres. Returns true if any hole along the way has no
   * traversable ground again within max_crossable_gap_ of where it started (no
   * reachable far side). Crossable gaps (a strip resumes within reach) and the
   * edge of the mapped area are passed over. Disabled (returns false) when
   * edge_clearance_ == 0 or max_crossable_gap_ == 0. This looks farther than
   * the NMPC horizon so LocalPlanner can start the graceful stop before the
   * robot commits to a field of narrow crossable gaps that ends at a cliff.
   *
   * @param[in] from Body xy position to probe from, world frame
   * @param[in] lookahead Distance to probe, metres
   * @return true if an uncrossable gap lies within `lookahead` ahead
   */
  bool hasUncrossableGapAhead(const Eigen::Vector2d& from,
                              double lookahead) const;

  /**
   * @brief Convert the foot positions and contact schedule into ros messages
   * for the foot plan
   * @param[in] contact_schedule Current contact schedule
   * @param[in] current_plan_index Current plan index
   * @param[in] first_element_duration Duration of first element of horizon (may
   * not be dt)
   * @param[in] foot_positions Foot positions over the horizon
   * @param[in] foot_velocities Foot velocities over the horizon
   * @param[in] foot_accelerations Foot accelerations over the horizon
   * @param[out] future_footholds_msg Message for future (planned) footholds
   * @param[out] foot_plan_continuous_msg Message for continuous foot
   * trajectories
   */
  void loadFootPlanMsgs(
      const std::vector<std::vector<bool>>& contact_schedule,
      int current_plan_index, double first_element_duration,
      const Eigen::MatrixXd& foot_positions,
      const Eigen::MatrixXd& foot_velocities,
      const Eigen::MatrixXd& foot_accelerations,
      quad_msgs::msg::MultiFootPlanDiscrete& future_footholds_msg,
      quad_msgs::msg::MultiFootPlanContinuous& foot_plan_continuous_msg);

  inline void printContactSchedule(
      const std::vector<std::vector<bool>>& contact_schedule) {
    for (size_t i = 0; i < contact_schedule.size(); i++) {
      for (size_t j = 0; j < contact_schedule.at(i).size(); j++) {
        if (contact_schedule[i][j]) {
          printf("1 ");
        } else {
          printf("0 ");
        }
      }
      printf("\n");
    }
  }

  inline double getTerrainHeight(double x, double y) {
    grid_map::Position pos = {x, y};
    double height = this->terrain_grid_.atPosition(
        "z_smooth", terrain_grid_.getClosestPositionInMap(pos),
        grid_map::InterpolationMethods::INTER_LINEAR);
    return (height);
  }

  inline double getTerrainSlope(double x, double y, double dx, double dy) {
    std::array<double, 3> surf_norm =
        this->terrain_.getSurfaceNormalFiltered(x, y);

    double denom = dx * dx + dy * dy;
    if (denom <= 0 || surf_norm[2] <= 0) {
      double default_pitch = 0;
      return default_pitch;
    } else {
      double v_proj = (dx * surf_norm[0] + dy * surf_norm[1]) / sqrt(denom);
      double pitch = atan2(v_proj, surf_norm[2]);

      return pitch;
    }
  }

  inline void getTerrainSlope(double x, double y, double yaw, double& roll,
                              double& pitch) {
    std::array<double, 3> surf_norm =
        this->terrain_.getSurfaceNormalFiltered(x, y);

    Eigen::Vector3d norm_vec(surf_norm.data());
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(norm_vec);
    double ang = acos(
        std::max(std::min(Eigen::Vector3d::UnitZ().dot(norm_vec), 1.), -1.));

    if (ang < 1e-3) {
      roll = 0;
      pitch = 0;
      return;
    } else {
      Eigen::Matrix3d rot(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
      axis = rot.transpose() * (axis / axis.norm());
      tf2::Quaternion quat(tf2::Vector3(axis(0), axis(1), axis(2)), ang);
      tf2::Matrix3x3 m(quat);
      double tmp;
      m.getRPY(roll, pitch, tmp);
    }
  }

  inline void setTerrainMap(const grid_map::GridMap& grid_map) {
    terrain_grid_ = grid_map;
  }

  // Extrapolate body translation while holding orientation fixed.
  inline Eigen::VectorXd computeFutureBodyPlan(
      double step, const Eigen::VectorXd& body_plan) {
    Eigen::VectorXd future_body_plan = body_plan;

    future_body_plan.segment(0, 3) =
        future_body_plan.segment(0, 3) + body_plan.segment(6, 3) * step * dt_;

    return future_body_plan;
  }

#ifdef LOCAL_PLANNER_TESTING
 public:
#else
 private:
#endif
  /**
   * @brief Update the continuous foot plan to match the discrete footholds
   */
  void updateContinuousPlan();

  /**
   * @brief Compute the cubic hermite spline
   * @param[in] pos_prev Previous position
   * @param[in] vel_prev Previous velocity
   * @param[in] pos_next Next position
   * @param[in] vel_next Next velocity
   * @param[in] phase Interpolation phase
   * @param[in] duration Interpolation duration
   * @param[out] pos Interpolated position
   * @param[out] vel Interpolated velocity
   * @param[out] acc Interpolated acceleration
   */
  void cubicHermiteSpline(double pos_prev, double vel_prev, double pos_next,
                          double vel_next, double phase, double duration,
                          double& pos, double& vel, double& acc);

  /**
   * @brief Search locally around a nominal foothold for valid terrain
   * @param[in] foot_position Foothold to optimize around
   * @param[in] foot_position_prev_solve Foothold in prior solve
   * @return Optimized foothold
   *
   * Thin wrapper over getNearestValidFootholdResult(); returns only .position
   * so existing callers are byte-for-byte unchanged.
   */
  Eigen::Vector3d getNearestValidFoothold(
      const Eigen::Vector3d& foot_position,
      const Eigen::Vector3d& foot_position_prev_solve) const;

  /**
   * @brief Same search as getNearestValidFoothold, but also reports the
   * outcome (FootholdStatus) and cheap diagnostics (FootholdResult).
   * Phase 1: the chosen .position is identical to getNearestValidFoothold's
   * return value in every case; only the extra fields are new.
   * @param[in] foot_position Nominal foothold to optimize around
   * @param[in] foot_position_prev_solve Foothold in prior solve
   * @param[in] leg_index Leg this foothold is for (Phase 4 reach check); -1 skips
   * @param[in] hip_world World-frame hip this foothold supports during stance
   * (the midstance hip from computeFootPlan) -- Phase 4 reach check
   * @return FootholdResult with position + status + diagnostics
   */
  FootholdResult getNearestValidFootholdResult(
      const Eigen::Vector3d& foot_position,
      const Eigen::Vector3d& foot_position_prev_solve, int leg_index = -1,
      const Eigen::Vector3d& hip_world = Eigen::Vector3d::Zero()) const;

  /**
   * @brief Compute the minimum enclosing circle using Welzl's algorithm
   * @param[in] P Hip position in the plan
   * @param[in] R Boundary points for the circle
   * @return Center and radius of the circle
   */
  Eigen::Vector3d welzlMinimumCircle(std::vector<Eigen::Vector2d> P,
                                     std::vector<Eigen::Vector2d> R);

  /**
   * @brief Compute swing apex height from terrain clearance and hip clearance
   * @param[in] leg_idx Leg index
   * @param[in] body_plan Body plan in the mid air index
   * @param[in] foot_position_prev Position of the previous foothold
   * @param[in] foot_position_next Position of the next foothold
   * @return Apex height
   */
  double computeSwingApex(int leg_idx, const Eigen::VectorXd& body_plan,
                          const Eigen::Vector3d& foot_position_prev,
                          const Eigen::Vector3d& foot_position_next);

  /**
   * @brief Extract foot data from the matrix
   */
  inline Eigen::Vector3d getFootData(const Eigen::MatrixXd& foot_state_vars,
                                     int horizon_index, int foot_index) {
    return foot_state_vars.block<1, 3>(horizon_index, 3 * foot_index);
  }

  /**
   * @brief Check if a foot is in contact at a given index
   */
  inline bool isContact(const std::vector<std::vector<bool>>& contact_schedule,
                        int horizon_index, int foot_index) {
    return (contact_schedule.at(horizon_index).at(foot_index));
  }

  /**
   * @brief Check if a foot is newly in contact at a given index
   */
  inline bool isNewContact(
      const std::vector<std::vector<bool>>& contact_schedule, int horizon_index,
      int foot_index) {
    if (horizon_index == 0) return false;

    return (!isContact(contact_schedule, horizon_index - 1, foot_index) &&
            isContact(contact_schedule, horizon_index, foot_index));
  }

  /**
   * @brief Check if a foot is newly in swing at a given index
   */
  inline bool isNewLiftoff(
      const std::vector<std::vector<bool>>& contact_schedule, int horizon_index,
      int foot_index) {
    if (horizon_index == 0) return false;

    return (isContact(contact_schedule, horizon_index - 1, foot_index) &&
            !isContact(contact_schedule, horizon_index, foot_index));
  }

  /**
   * @brief Compute the index of the next contact for a foot. If none exist
   * return the last.
   */
  inline int getNextContactIndex(
      const std::vector<std::vector<bool>>& contact_schedule, int horizon_index,
      int foot_index) {
    for (int i_touchdown = horizon_index; i_touchdown < horizon_length_;
         i_touchdown++) {
      if (isNewContact(contact_schedule, i_touchdown, foot_index)) {
        return i_touchdown;
      }
    }

    return (horizon_length_ - 1);
  }

  /**
   * @brief Compute the index of the next liftoff for a foot. If none exist
   * return the last.
   */
  inline int getNextLiftoffIndex(
      const std::vector<std::vector<bool>>& contact_schedule, int horizon_index,
      int foot_index) {
    for (int i_liftoff = horizon_index; i_liftoff < horizon_length_;
         i_liftoff++) {
      if (isNewLiftoff(contact_schedule, i_liftoff, foot_index)) {
        return i_liftoff;
      }
    }

    return (horizon_length_ - 1);
  }

  /// Shared Pointer to Node
  rclcpp::Node::SharedPtr node_;

  /// Struct for terrain map data
  FastTerrainMap terrain_;

  /// GridMap for terrain map data
  grid_map::GridMap terrain_grid_;

  /// Number of feet
  const int num_feet_ = 4;

  /// Timestep for one finite element
  double dt_;

  /// Gait period in timesteps
  int period_;

  /// Horizon length in timesteps
  int horizon_length_;

  /// Phase offsets for the touchdown of each foot
  std::vector<double> phase_offsets_ = {0, 0.5, 0.5, 0.0};

  /// Duty cycles for the stance duration of each foot
  std::vector<double> duty_cycles_ = {0.5, 0.5, 0.5, 0.5};

  /// Nominal contact schedule
  std::vector<std::vector<bool>> nominal_contact_schedule_;

  /// Ground clearance
  double ground_clearance_;

  /// Hip clearance
  double hip_clearance_;

  /// Weighting on GRF-based foothold adjustment
  double grf_weight_;

  /// Primitive ids (values from quad_utils::PrimitiveId, the single source of
  /// truth shared with the global body planner and rviz_interface). static
  /// constexpr so they can be used as switch case labels.
  static constexpr int CONNECT_STANCE = quad_utils::PRIM_CONNECT;
  static constexpr int LEAP_STANCE = quad_utils::PRIM_LEAP_STANCE;
  static constexpr int FLIGHT = quad_utils::PRIM_FLIGHT;
  static constexpr int LAND_STANCE = quad_utils::PRIM_LAND_STANCE;

  /// Step 17 forward-jump sub-phases
  static constexpr int PRELOAD = quad_utils::PRIM_PRELOAD;
  static constexpr int REAR_PUSH = quad_utils::PRIM_REAR_PUSH;
  static constexpr int FRONT_LAND = quad_utils::PRIM_FRONT_LAND;
  static constexpr int SETTLE = quad_utils::PRIM_SETTLE;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD2> quadKD_;

  /// Threshold of body error from desired goal to start stepping
  double standing_error_threshold_ = 0;

  /// Radius to locally search for valid footholds (m)
  double foothold_search_radius_;

  /// Minimum objective function value for valid foothold
  double foothold_obj_threshold_;

  /// Terrain layer for foothold search
  std::string obj_fun_layer_;

  /// Toe radius
  double toe_radius_;

  /// Phase 3: required clear distance from a chosen foothold to the nearest
  /// non-traversable / off-map cell, in metres. 0 disables the check (footholds
  /// may sit right on a hole lip, the pre-Phase-3 behaviour); > 0 marks a
  /// foothold with an unsafe cell within this radius as EDGE_TOO_CLOSE.
  double edge_clearance_ = 0.0;

  /// Phase 3 (crossability): when an EDGE_TOO_CLOSE foothold has traversable
  /// ground again within this distance ahead (past the hole), the hole is
  /// treated as a crossable gap and the status is put back to VALID. Only holes
  /// with no far side within this reach keep EDGE_TOO_CLOSE. Metres.
  double max_crossable_gap_ = 0.6;

  /// Phase 4: when true, a VALID foothold whose distance from the leg's nominal
  /// hip (at the predicted touchdown body pose) exceeds ik_max_reach_ is
  /// downgraded to IK_UNREACHABLE. false disables the check (pre-Phase-4
  /// behaviour); needs a kinematics object.
  bool ik_reach_check_ = false;
  /// Phase 4: max hip-to-foothold distance treated as reachable, metres.
  double ik_max_reach_ = 0.45;

  /// [MPC_DOG Step 14] multi-step foothold-sequence shadow planner config.
  bool multistep_enabled_ = false;
  bool multistep_apply_stop_ = false;
  int multistep_stop_margin_steps_ = 4;
  double multistep_planning_distance_ = 2.5;

  /// [MPC_DOG Step 15] feed the planned foothold sequence to the nominal.
  bool multistep_apply_foothold_ = false;
  /// latest planned first-touchdown foothold per leg index: world x/y and the
  /// body x it was planned for. Only applied to an actual touchdown whose
  /// predicted body x matches multistep_planned_bx_ (so the world position is
  /// current, not stale), as a clamped correction. Refreshed every 5th cycle.
  Eigen::Vector2d multistep_planned_xy_[4] = {
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero()};
  double multistep_planned_bx_[4] = {0.0, 0.0, 0.0, 0.0};
  bool multistep_planned_ok_[4] = {false, false, false, false};
  int multistep_planned_plan_index_ = -1;
};

#endif  // LOCAL_FOOTSTEP_PLANNER_H
