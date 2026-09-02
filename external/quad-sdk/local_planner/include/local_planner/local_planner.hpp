#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <gtest/gtest_prod.h>
#include <local_planner/local_footstep_planner.hpp>
#include <local_planner/local_planner_modes.hpp>
#include <math.h>
#include <nmpc_controller/nmpc_controller.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <quad_msgs/msg/multi_foot_plan_discrete.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/ros_utils.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//! Local Body Planner library
/*!
   Wrapper around Quadrupedal MPC that interfaces with our ROS architecture
*/
class LocalPlanner {
 public:
  /**
   * @brief Constructor for LocalPlanner
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type LocalPlanner
   */
  LocalPlanner(rclcpp::Node::SharedPtr node);

  /**
   * @brief Primary work function in class, called in node file for this
   * component
   */
  void spin();

#ifdef LOCAL_PLANNER_TESTING
 public:
#else
 private:
#endif
  FRIEND_TEST(LocalPlannerTest, noInputCase);

  /**
   * @brief Initialize the local body planner
   */
  void initLocalBodyPlanner();

  /**
   * @brief Initialize the local footstep planner
   */
  void initLocalFootstepPlanner();

  /**
   * @brief Callback function to handle new terrain map data
   * @param[in] grid_map_msgs::GridMap contining map data
   */
  void terrainMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  /**
   * @brief Callback function to handle new plans
   * @param[in] msg Robot state trajectory message
   */
  void robotPlanCallback(const quad_msgs::msg::RobotPlan::SharedPtr msg);

  /**
   * @brief Callback function to handle new state estimates
   * @param[in] State estimate message contining position and velocity for each
   * joint and robot body
   */
  void robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Callback function to handle new desired twist data when using twist
   * input
   * @param[in] msg the message contining twist data
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Function to compute reference trajectory from twist command or
   * global plan
   */
  void getReference();

  /**
   * @brief Unwrap the yaw signal in the reference body plan
   */
  void unwrapYawReference();

  /**
   * @brief Function to compute the local plan
   * @return Boolean if local plan was found successfully
   */
  bool computeLocalPlan();

  /**
   * @brief Function to publish the local plan
   */
  void publishLocalPlan();

  /// Robot type: A1 or Spirit
  std::string robot_name_;

  /// Robot Namespace: robot_1, ...
  std::string robot_ns_;

  /// Robot Description urdf
  std::string robot_description_;

  /// ROS subscriber for incoming terrain_map
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_sub_;

  /// ROS subscriber for incoming body plans
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr body_plan_sub_;

  /// ROS Subscriber for incoming states
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

  /// Subscriber for twist input messages
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// ROS publisher for local plan output
  rclcpp::Publisher<quad_msgs::msg::RobotPlan>::SharedPtr local_plan_pub_;

  /// ROS publisher for discrete foot plan
  rclcpp::Publisher<quad_msgs::msg::MultiFootPlanDiscrete>::SharedPtr
      foot_plan_discrete_pub_;

  /// ROS publisher for continuous foot plan
  rclcpp::Publisher<quad_msgs::msg::MultiFootPlanContinuous>::SharedPtr
      foot_plan_continuous_pub_;

  /// ROS publisher for planner failure signal
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_failed_pub_;

  /// Count of consecutive MPC solve failures
  int consecutive_failures_ = 0;

  /// Threshold of consecutive failures before publishing
  int failure_threshold_ = 10;

  /// Phase 2A: when true, computeLocalPlan() withholds the local plan (returns
  /// false, so publishLocalPlan() is not called and the plan goes stale ->
  /// robot_driver PD-holds the stand pose) if computeFootPlan() could not place
  /// every touchdown on a traversable in-map cell. Set false to fall back to the
  /// pre-Phase-2A behaviour (plan with the last-good foothold held).
  bool stop_on_invalid_foothold_ = true;

  /// Phase 2B: when true, an invalid foothold at horizon index <=
  /// safe_stop_horizon_ latches safe_stop_latched_ instead of freezing the
  /// plan. While latched, getReference() forces cmd_vel_ to zero so the
  /// existing STEP->STAND transition decelerates the body, lands the swing
  /// legs and holds a stand pose -- and the plan keeps being published. Set
  /// false for the Phase 2A behaviour (withhold the plan -> PD hold).
  bool safe_stop_latch_ = true;
  /// Only invalid footholds within this many horizon steps latch the stop
  /// (default = whole horizon). Farther-out invalid footholds are ignored
  /// (Phase 2A-3 already replaces them with the previous foothold).
  int safe_stop_horizon_ = 40;
  /// Phase 2B-3: metres to probe the terrain map forward from the body for an
  /// uncrossable gap (independent of the NMPC horizon). Lets the graceful stop
  /// latch before the robot enters a field of crossable gaps that ends at a
  /// cliff. 0 disables this body-forward check.
  double safe_stop_lookahead_ = 2.5;
  /// Latched once an impassable gap is seen in the horizon; stays latched for
  /// the rest of the WALK session.
  bool safe_stop_latched_ = false;

  /// [MPC_DOG Step 14] multi-step foothold-sequence planner -> control link.
  /// enabled: run the shadow search; apply_stop_request: act on its verdict
  /// (latch the Phase 2B graceful stop on STOP_REQUEST, scale cmd_vel on SLOW).
  /// Both default false = pre-Step-14 behaviour.
  bool multistep_enabled_ = false;
  bool multistep_apply_stop_ = false;
  /// [MPC_DOG Step 15] apply_foothold: feed the planned foothold sequence to the
  /// nominal (receding horizon, nearest touchdown per leg; existing snap still
  /// runs as the final local correction). default false = pre-Step-15 behaviour.
  bool multistep_apply_foothold_ = false;
  int multistep_stop_margin_steps_ = 4;
  double multistep_planning_distance_ = 2.5;
  double multistep_slow_factor_ = 0.4;
  /// Set each cycle from FootPlanResult::multistep_slow; getReference() scales
  /// cmd_vel by multistep_slow_factor_ while it is true and not yet latched.
  bool multistep_slow_active_ = false;

  /// Define map frame
  std::string map_frame_;

  /// Nodehandle to pub to and sub from
  rclcpp::Node::SharedPtr node_;

  /// Struct for terrain map data
  FastTerrainMap terrain_;

  /// GridMap for terrain map data
  grid_map::GridMap terrain_grid_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Local Body Planner object
  std::shared_ptr<NMPCController> local_body_planner_nonlinear_;

  /// Local Footstep Planner object
  std::shared_ptr<LocalFootstepPlanner> local_footstep_planner_;

  /// Most recent robot plan
  quad_msgs::msg::RobotPlan::SharedPtr body_plan_msg_;

  /// Most recent robot state
  quad_msgs::msg::RobotState::SharedPtr robot_state_msg_;

  /// Past foothold locations
  quad_msgs::msg::MultiFootState past_footholds_msg_;

  /// Timestamp of the state estimate
  rclcpp::Time current_state_timestamp_;

  /// Current state (ground truth or estimate)
  Eigen::VectorXd current_state_;

  // Current positions of each foot
  Eigen::VectorXd current_foot_positions_world_;

  // Current velocities of each foot
  Eigen::VectorXd current_foot_velocities_world_;

  // Current positions of each foot
  Eigen::VectorXd current_foot_positions_body_;

  /// Current index in the global plan
  int current_plan_index_;

  /// local planner timestep (seconds)
  double dt_;

  /// Computation time in computeLocalPlan
  double compute_time_;

  /// Average computation time in computeLocalPlan
  double mean_compute_time_;

  /// Exponential filter smoothing constant (higher updates slower)
  const double filter_smoothing_constant_ = 0.5;

  /// Standard MPC horizon length
  int N_;

  /// Current MPC horizon length
  int N_current_;

  /// Number of states
  const int Nx_ = 12;

  /// Number of controls
  const int Nu_ = 13;

  /// Number of legs
  const int num_feet_ = 4;

  /// Number of joints per leg
  const int num_joints_per_leg_ = 3;

  /// Matrix of body states (N x Nx: rows correspond to individual states in the
  /// horizon)
  Eigen::MatrixXd body_plan_;

  /// Matrix of body states (N x Nx: rows correspond to individual states in the
  /// horizon)
  Eigen::MatrixXd ref_body_plan_;

  /// Vector of ground height along reference trajectory
  Eigen::VectorXd ref_ground_height_;

  /// Vector of primitive along reference trajectory
  Eigen::VectorXi ref_primitive_plan_;

  /// Matrix of grfs (N x Nu: rows correspond to individual arrays of GRFs in
  /// the horizon)
  Eigen::MatrixXd grf_plan_;

  /// Contact schedule
  std::vector<std::vector<bool>> contact_schedule_;

  /// Matrix of continuous foot positions in world frame
  Eigen::MatrixXd foot_positions_world_;

  /// Matrix of continuous foot velocities in world frame
  Eigen::MatrixXd foot_velocities_world_;

  /// Matrix of continuous foot accelerations in world frame
  Eigen::MatrixXd foot_accelerations_world_;

  /// Matrix of continuous foot positions in body frame
  Eigen::MatrixXd foot_positions_body_;

  /// Matrix of foot contact locations (number of contacts x num_legs_)
  Eigen::MatrixXd foot_plan_discrete_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD2> quadKD_;

  /// Twist input
  Eigen::VectorXd cmd_vel_;

  /// Commanded velocity filter constant
  double cmd_vel_filter_const_;

  /// Scale for twist cmd_vel
  double cmd_vel_scale_;

  /// Nominal robot height
  double z_des_;

  /// Time of the most recent cmd_vel data
  rclcpp::Time last_cmd_vel_msg_time_;

  /// Threshold for waiting for twist cmd_vel data
  double last_cmd_vel_msg_time_max_;

  /// Initial timestamp for contact cycling
  rclcpp::Time initial_timestamp_;

  /// Foot initialization flag when using twist input without a global body plan
  bool first_plan_;

  /// Boolean for using twist input instead of a global body plan
  bool use_twist_input_;

  /// Vector for stand pose (x, y, yaw)
  Eigen::Vector3d stand_pose_;

  /// Time duration to the next plan index
  double first_element_duration_;

  /// Difference in plan index from last solve
  int plan_index_diff_;

  /// Toe radius
  double toe_radius_;

  /// Control mode
  int control_mode_;

  /// Velocity threshold to enter stand mode
  double stand_vel_threshold_;

  /// Commanded velocity threshold to enter stand mode
  double stand_cmd_vel_threshold_;

  /// Position error threshold (from foot centroid) to enter stand mode
  double stand_pos_error_threshold_;
};

#endif  // LOCAL_PLANNER_H
