#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planner/local_footstep_planner.hpp"

namespace {

constexpr double kTol = 1e-6;

std::string runXacro(const std::string& xacro_path) {
  std::array<char, 4096> buffer{};
  std::string result;
  const std::string cmd = "xacro " + xacro_path;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    throw std::runtime_error("Failed to run xacro");
  }
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  const int rc = pclose(pipe);
  if (rc != 0 || result.empty()) {
    throw std::runtime_error("xacro returned no robot description");
  }
  return result;
}

std::string go2RobotDescription() {
  static const std::string urdf = []() {
    const char* source_dir = std::getenv("LOCAL_PLANNER_SOURCE_DIR");
    if (source_dir == nullptr) {
      throw std::runtime_error("Missing local_planner source env");
    }
    return runXacro(std::string(source_dir) +
                    "/quad_simulator/go2_description/models/go2/urdf/"
                    "go2.urdf.xacro");
  }();
  return urdf;
}

grid_map::GridMap makeTerrain(double height = 0.0, double traversability = 1.0) {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(4.0, 4.0), 0.1);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("z_inpainted", *it) = height;
    map.at("z_smooth", *it) = height;
    map.at("normal_vectors_x", *it) = 0.0;
    map.at("normal_vectors_y", *it) = 0.0;
    map.at("normal_vectors_z", *it) = 1.0;
    map.at("smooth_normal_vectors_x", *it) = 0.0;
    map.at("smooth_normal_vectors_y", *it) = 0.0;
    map.at("smooth_normal_vectors_z", *it) = 1.0;
    map.at("traversability", *it) = traversability;
  }
  return map;
}

std::shared_ptr<rclcpp::Node> makeGo2Node(const std::string& name) {
  const char* robot_params = std::getenv("LOCAL_PLANNER_ROBOT_PARAMS");
  if (robot_params == nullptr) {
    throw std::runtime_error("Missing local_planner robot parameter env");
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", robot_params});
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
      rclcpp::Parameter("namespace", "robot_1"),
      rclcpp::Parameter("robot_type", "go2"),
      rclcpp::Parameter("robot_description", go2RobotDescription()),
  });
  return std::make_shared<rclcpp::Node>(name, options);
}

LocalFootstepPlanner makePlanner(double edge_clearance = 0.0,
                                 double max_crossable_gap = 0.0) {
  auto node = std::make_shared<rclcpp::Node>("local_footstep_planner_test");
  LocalFootstepPlanner planner(node);
  planner.setTemporalParams(0.1, 4, 6, {0.5, 0.5, 0.5, 0.5},
                            {0.0, 0.5, 0.5, 0.0});
  planner.setSpatialParams(0.07, 0.1, 0.45, 0.03, nullptr, 0.25, 0.6,
                           "traversability", 0.02, edge_clearance,
                           max_crossable_gap);
  const auto terrain_grid = makeTerrain();
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(terrain_grid);
  planner.updateMap(terrain_grid);
  planner.updateMap(terrain);
  return planner;
}

LocalFootstepPlanner makeGo2Planner() {
  auto node = makeGo2Node("local_footstep_planner_go2_test");
  auto kinematics = std::make_shared<quad_utils::QuadKD2>(node, "robot_1");
  LocalFootstepPlanner planner(node);
  planner.setTemporalParams(0.1, 4, 6, {0.5, 0.5, 0.5, 0.5},
                            {0.0, 0.5, 0.5, 0.0});
  planner.setSpatialParams(0.07, 0.1, 0.45, 0.03, kinematics, 0.25, 0.6,
                           "traversability", 0.02);
  const auto terrain_grid = makeTerrain();
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(terrain_grid);
  planner.updateMap(terrain_grid);
  planner.updateMap(terrain);
  return planner;
}

// Traversable everywhere, but z_inpainted left as NaN (never assigned).
grid_map::GridMap makeTerrainNonfiniteHeight() {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(4.0, 4.0), 0.1);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("z_smooth", *it) = 0.0;
    map.at("normal_vectors_x", *it) = 0.0;
    map.at("normal_vectors_y", *it) = 0.0;
    map.at("normal_vectors_z", *it) = 1.0;
    map.at("smooth_normal_vectors_x", *it) = 0.0;
    map.at("smooth_normal_vectors_y", *it) = 0.0;
    map.at("smooth_normal_vectors_z", *it) = 1.0;
    map.at("traversability", *it) = 1.0;
    // z_inpainted deliberately left NaN
  }
  return map;
}

// A circular non-traversable hole of the given radius centred at the origin;
// everything outside is traversable and flat at z = 0.
grid_map::GridMap makeTerrainWithHole(double hole_radius) {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(4.0, 4.0), 0.1);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position pos;
    map.getPosition(*it, pos);
    const bool in_hole = pos.norm() < hole_radius;
    map.at("z_inpainted", *it) = 0.0;
    map.at("z_smooth", *it) = 0.0;
    map.at("normal_vectors_x", *it) = 0.0;
    map.at("normal_vectors_y", *it) = 0.0;
    map.at("normal_vectors_z", *it) = 1.0;
    map.at("smooth_normal_vectors_x", *it) = 0.0;
    map.at("smooth_normal_vectors_y", *it) = 0.0;
    map.at("smooth_normal_vectors_z", *it) = 1.0;
    map.at("traversability", *it) = in_hole ? 0.0 : 1.0;
  }
  return map;
}

// Flat z=0 terrain, traversable everywhere except a non-traversable band
// x in [gap_lo, gap_hi] (a straight gap with solid ground on both sides).
grid_map::GridMap makeTerrainWithGapBand(double gap_lo, double gap_hi) {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(6.0, 4.0), 0.1);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position pos;
    map.getPosition(*it, pos);
    const bool in_gap = pos.x() >= gap_lo && pos.x() <= gap_hi;
    map.at("z_inpainted", *it) = 0.0;
    map.at("z_smooth", *it) = 0.0;
    map.at("normal_vectors_x", *it) = 0.0;
    map.at("normal_vectors_y", *it) = 0.0;
    map.at("normal_vectors_z", *it) = 1.0;
    map.at("smooth_normal_vectors_x", *it) = 0.0;
    map.at("smooth_normal_vectors_y", *it) = 0.0;
    map.at("smooth_normal_vectors_z", *it) = 1.0;
    map.at("traversability", *it) = in_gap ? 0.0 : 1.0;
  }
  return map;
}

void setFoot(quad_msgs::msg::MultiFootState& feet, int foot, double x, double y,
             double z, int traj_index) {
  feet.feet[foot].position.x = x;
  feet.feet[foot].position.y = y;
  feet.feet[foot].position.z = z;
  feet.feet[foot].traj_index = traj_index;
}

}  // namespace

TEST(LocalFootstepPlannerTest, ContactScheduleTilesGaitAndStandMode) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  Eigen::VectorXi primitives = Eigen::VectorXi::Zero(6);
  std::vector<std::vector<bool>> schedule;

  planner.computeContactSchedule(0, body_plan, primitives, STEP, schedule);

  ASSERT_EQ(schedule.size(), 6u);
  EXPECT_EQ(schedule[0], (std::vector<bool>{true, false, false, true}));
  EXPECT_EQ(schedule[1], (std::vector<bool>{true, false, false, true}));
  EXPECT_EQ(schedule[2], (std::vector<bool>{false, true, true, false}));
  EXPECT_EQ(schedule[3], (std::vector<bool>{false, true, true, false}));
  EXPECT_EQ(schedule[4], schedule[0]);

  planner.computeContactSchedule(0, body_plan, primitives, STAND, schedule);
  for (const auto& row : schedule) {
    EXPECT_EQ(row, (std::vector<bool>{true, true, true, true}));
  }
}

TEST(LocalFootstepPlannerTest, ContactScheduleOverridesFlightAndLanding) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  Eigen::VectorXi primitives = Eigen::VectorXi::Zero(6);
  primitives(1) = 2;
  primitives(2) = 3;
  std::vector<std::vector<bool>> schedule;

  planner.computeContactSchedule(0, body_plan, primitives, STEP, schedule);

  EXPECT_EQ(schedule[1], (std::vector<bool>{false, false, false, false}));
  EXPECT_EQ(schedule[2], (std::vector<bool>{true, true, true, true}));
}

TEST(LocalFootstepPlannerTest, FootPositionsTransformFromWorldToBody) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::VectorXd body = Eigen::VectorXd::Zero(12);
  body << 1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd feet_world(12);
  feet_world << 1.2, 2.3, 0.0, 0.8, 2.3, 0.0, 1.2, 1.7, 0.0, 0.8, 1.7, 0.0;
  Eigen::VectorXd feet_body = Eigen::VectorXd::Zero(12);

  planner.getFootPositionsBodyFrame(body, feet_world, feet_body);

  EXPECT_NEAR(feet_body[0], 0.2, kTol);
  EXPECT_NEAR(feet_body[1], 0.3, kTol);
  EXPECT_NEAR(feet_body[2], -0.5, kTol);

  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  body_plan.row(0) = body;
  body_plan.row(1) = body;
  Eigen::MatrixXd feet_plan(6, 12);
  feet_plan.row(0) = feet_world;
  feet_plan.row(1) = feet_world;
  Eigen::MatrixXd feet_body_plan = Eigen::MatrixXd::Zero(6, 12);

  planner.getFootPositionsBodyFrame(body_plan, feet_plan, feet_body_plan);

  EXPECT_NEAR(feet_body_plan(1, 0), 0.2, kTol);
  EXPECT_NEAR(feet_body_plan(1, 2), -0.5, kTol);
}

TEST(LocalFootstepPlannerTest, TerrainQueriesAndFutureBodyPlan) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::VectorXd body = Eigen::VectorXd::Zero(12);
  body[0] = 1.0;
  body[1] = 2.0;
  body[2] = 0.3;
  body[6] = 0.5;
  body[7] = -0.2;

  const Eigen::VectorXd future = planner.computeFutureBodyPlan(3.0, body);

  EXPECT_NEAR(planner.getTerrainHeight(0.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(planner.getTerrainSlope(0.0, 0.0, 1.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(future[0], 1.15, kTol);
  EXPECT_NEAR(future[1], 1.94, kTol);
  EXPECT_NEAR(future[5], body[5], kTol);
}

TEST(LocalFootstepPlannerTest, CubicHermiteSplineMatchesEndpoints) {
  LocalFootstepPlanner planner = makePlanner();
  double pos = 0.0;
  double vel = 0.0;
  double acc = 0.0;

  planner.cubicHermiteSpline(1.0, 0.0, 3.0, 0.0, 0.0, 2.0, pos, vel, acc);
  EXPECT_NEAR(pos, 1.0, kTol);
  EXPECT_NEAR(vel, 0.0, kTol);

  planner.cubicHermiteSpline(1.0, 0.0, 3.0, 0.0, 1.0, 2.0, pos, vel, acc);
  EXPECT_NEAR(pos, 3.0, kTol);
  EXPECT_NEAR(vel, 0.0, kTol);
}

TEST(LocalFootstepPlannerTest, ContactTransitionHelpersFindEdges) {
  LocalFootstepPlanner planner = makePlanner();
  std::vector<std::vector<bool>> schedule = {
      {true, false, true, true},  {false, false, true, true},
      {false, true, true, true},  {true, true, false, true},
      {true, true, true, true},   {true, false, true, true},
  };

  EXPECT_FALSE(planner.isNewLiftoff(schedule, 0, 0));
  EXPECT_TRUE(planner.isNewLiftoff(schedule, 1, 0));
  EXPECT_TRUE(planner.isNewContact(schedule, 2, 1));
  EXPECT_EQ(planner.getNextContactIndex(schedule, 0, 0), 3);
  EXPECT_EQ(planner.getNextLiftoffIndex(schedule, 0, 2), 3);
  EXPECT_EQ(planner.getNextContactIndex(schedule, 4, 0), 5);
}

TEST(LocalFootstepPlannerTest, FootholdSearchUsesValidTerrainAndToeRadius) {
  LocalFootstepPlanner planner = makePlanner();
  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);

  const Eigen::Vector3d foothold =
      planner.getNearestValidFoothold(nominal, previous);

  EXPECT_NEAR(foothold.x(), 0.0, 0.11);
  EXPECT_NEAR(foothold.y(), 0.0, 0.11);
  EXPECT_NEAR(foothold.z(), 0.02, kTol);
}

TEST(LocalFootstepPlannerTest, FootholdSearchFallsBackWhenTerrainInvalid) {
  LocalFootstepPlanner planner = makePlanner();
  const auto invalid_grid = makeTerrain(0.15, 0.0);
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(invalid_grid);
  planner.updateMap(invalid_grid);
  planner.updateMap(terrain);

  const Eigen::Vector3d nominal(0.32, -0.18, 0.0);
  const Eigen::Vector3d previous(-0.5, 0.5, 0.0);
  const Eigen::Vector3d foothold =
      planner.getNearestValidFoothold(nominal, previous);

  EXPECT_NEAR(foothold.x(), nominal.x(), kTol);
  EXPECT_NEAR(foothold.y(), nominal.y(), kTol);
  EXPECT_NEAR(foothold.z(), 0.17, kTol);
}

// ---- Phase 1: FootholdResult diagnostics (behaviour unchanged) ----

TEST(LocalFootstepPlannerTest, FootholdResultReportsValidOnFlatTerrain) {
  LocalFootstepPlanner planner = makePlanner();
  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);

  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, previous);

  EXPECT_EQ(r.status, FootholdStatus::VALID);
  EXPECT_NEAR(r.snap_distance, 0.0, 0.11);
  EXPECT_NEAR(r.traversability_nominal, 1.0, kTol);
  EXPECT_NEAR(r.traversability_selected, 1.0, kTol);
  EXPECT_NEAR(r.position.z(), 0.02, kTol);
  // Wrapper and result agree on the chosen position.
  const Eigen::Vector3d pos = planner.getNearestValidFoothold(nominal, previous);
  EXPECT_NEAR(pos.x(), r.position.x(), kTol);
  EXPECT_NEAR(pos.y(), r.position.y(), kTol);
  EXPECT_NEAR(pos.z(), r.position.z(), kTol);
}

TEST(LocalFootstepPlannerTest, FootholdResultReportsNoTraversableCandidate) {
  LocalFootstepPlanner planner = makePlanner();
  const auto invalid_grid = makeTerrain(0.15, 0.0);  // traversability 0 all over
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(invalid_grid);
  planner.updateMap(invalid_grid);
  planner.updateMap(terrain);

  const Eigen::Vector3d nominal(0.32, -0.18, 0.0);
  const Eigen::Vector3d previous(-0.5, 0.5, 0.0);
  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, previous);

  EXPECT_EQ(r.status, FootholdStatus::NO_TRAVERSABLE_CANDIDATE);
  // Behaviour unchanged: falls back to the nominal x/y, z from z_inpainted.
  EXPECT_NEAR(r.position.x(), nominal.x(), kTol);
  EXPECT_NEAR(r.position.y(), nominal.y(), kTol);
  EXPECT_NEAR(r.position.z(), 0.17, kTol);
  EXPECT_NEAR(r.snap_distance, 0.0, kTol);
}

TEST(LocalFootstepPlannerTest, FootholdResultReportsNominalOutsideMap) {
  LocalFootstepPlanner planner = makePlanner();
  const Eigen::Vector3d nominal(10.0, 10.0, 0.0);  // map is only 4 x 4 m
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);

  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, previous);

  EXPECT_EQ(r.status, FootholdStatus::NOMINAL_OUTSIDE_MAP);
  EXPECT_NEAR(r.position.x(), nominal.x(), kTol);
  EXPECT_NEAR(r.position.y(), nominal.y(), kTol);
}

TEST(LocalFootstepPlannerTest, FootholdResultReportsNonfiniteHeight) {
  LocalFootstepPlanner planner = makePlanner();
  const auto grid = makeTerrainNonfiniteHeight();
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(grid);
  planner.updateMap(grid);
  planner.updateMap(terrain);

  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);
  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, previous);

  EXPECT_EQ(r.status, FootholdStatus::NONFINITE_HEIGHT);
  EXPECT_FALSE(std::isfinite(r.position.z()));
}

TEST(LocalFootstepPlannerTest, FootholdResultSnapDistanceMatchesSelection) {
  LocalFootstepPlanner planner = makePlanner();
  // Widen the search radius so a valid cell outside the hole is reachable.
  planner.setSpatialParams(0.07, 0.1, 0.45, 0.03, nullptr, 0.6, 0.6,
                           "traversability", 0.02);
  const auto grid = makeTerrainWithHole(0.25);
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(grid);
  planner.updateMap(grid);
  planner.updateMap(terrain);

  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);
  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, previous);

  EXPECT_EQ(r.status, FootholdStatus::VALID);
  EXPECT_GT(r.snap_distance, 0.0);
  const double d =
      (r.position.head<2>() - nominal.head<2>()).norm();
  EXPECT_NEAR(r.snap_distance, d, kTol);
  EXPECT_NEAR(r.traversability_selected, 1.0, kTol);
}

// ---- Phase 3: EDGE_TOO_CLOSE (forward probe: lip before an uncrossable gap) --

// makeEdgeGrid: flat terrain with a non-traversable band x in [lo, hi].
static FastTerrainMap loadTerrain(const grid_map::GridMap& grid) {
  FastTerrainMap t;
  t.loadDataFromGridMap(grid);
  return t;
}

// A foothold just before a WIDE gap (no far side within max_crossable_gap ahead)
// is EDGE_TOO_CLOSE, and edge_clearance records how far ahead the hole starts.
TEST(LocalFootstepPlannerTest, EdgeTooCloseForLipBeforeUncrossableGap) {
  LocalFootstepPlanner planner = makePlanner(0.15, 0.6);
  const auto grid = makeTerrainWithGapBand(0.05, 1.5);  // 1.45 m gap ahead in x
  planner.updateMap(grid);
  planner.updateMap(loadTerrain(grid));

  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);  // hole starts ~0.05 m ahead
  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, nominal);

  EXPECT_EQ(r.status, FootholdStatus::EDGE_TOO_CLOSE);
  EXPECT_GT(r.edge_clearance, 0.0);
  EXPECT_LE(r.edge_clearance, 0.15);
}

// A foothold on the near lip of a CROSSABLE gap (solid ground resumes within
// max_crossable_gap ahead, as in step03/04's 0.3 m trench) stays VALID.
TEST(LocalFootstepPlannerTest, ValidForLipBeforeCrossableGap) {
  LocalFootstepPlanner planner = makePlanner(0.15, 0.6);
  const auto grid = makeTerrainWithGapBand(0.05, 0.35);  // 0.30 m gap ahead
  planner.updateMap(grid);
  planner.updateMap(loadTerrain(grid));

  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);
  const FootholdResult r =
      planner.getNearestValidFootholdResult(nominal, nominal);
  EXPECT_EQ(r.status, FootholdStatus::VALID);
}

// A hole that is behind the foothold, or further ahead than edge_clearance, or
// with edge_clearance disabled (== 0), all leave the foothold VALID.
TEST(LocalFootstepPlannerTest, EdgeProbeIgnoresHoleBehindFarAndWhenDisabled) {
  const auto wide = makeTerrainWithGapBand(0.05, 1.5);

  // Hole is 0.4 m ahead -> beyond edge_clearance 0.15 -> VALID.
  LocalFootstepPlanner far_planner = makePlanner(0.15, 0.6);
  const auto far_grid = makeTerrainWithGapBand(0.4, 1.9);
  far_planner.updateMap(far_grid);
  far_planner.updateMap(loadTerrain(far_grid));
  EXPECT_EQ(far_planner
                .getNearestValidFootholdResult(Eigen::Vector3d(0.0, 0.0, 0.0),
                                               Eigen::Vector3d(0.0, 0.0, 0.0))
                .status,
            FootholdStatus::VALID);

  // Same wide hole but it is BEHIND the foothold (foothold at x = 1.7, gap
  // [0.05, 1.5]) -> the +x probe sees only solid ground -> VALID.
  LocalFootstepPlanner behind_planner = makePlanner(0.15, 0.6);
  behind_planner.updateMap(wide);
  behind_planner.updateMap(loadTerrain(wide));
  EXPECT_EQ(behind_planner
                .getNearestValidFootholdResult(Eigen::Vector3d(1.7, 0.0, 0.0),
                                               Eigen::Vector3d(1.7, 0.0, 0.0))
                .status,
            FootholdStatus::VALID);

  // edge_clearance == 0 disables the check entirely.
  LocalFootstepPlanner off_planner = makePlanner(0.0, 0.6);
  off_planner.updateMap(wide);
  off_planner.updateMap(loadTerrain(wide));
  EXPECT_EQ(off_planner
                .getNearestValidFootholdResult(Eigen::Vector3d(0.0, 0.0, 0.0),
                                               Eigen::Vector3d(0.0, 0.0, 0.0))
                .status,
            FootholdStatus::VALID);

  // Foothold near the +x edge of the map (map is x in [-3, 3], no gap): the
  // probe reaches the map boundary and stops without flagging -- the unmapped
  // area beyond is not a cliff. (Phase 2B: this removed spurious EDGE_TOO_CLOSE
  // on far-horizon footholds near the last mapped strip.)
  const auto flat = makeTerrainWithGapBand(-2.9, -2.8);  // trivial band, far away
  LocalFootstepPlanner edge_planner = makePlanner(0.15, 0.6);
  edge_planner.updateMap(flat);
  edge_planner.updateMap(loadTerrain(flat));
  EXPECT_EQ(edge_planner
                .getNearestValidFootholdResult(Eigen::Vector3d(2.8, 0.0, 0.0),
                                               Eigen::Vector3d(2.8, 0.0, 0.0))
                .status,
            FootholdStatus::VALID);
}

TEST(LocalFootstepPlannerTest, WelzlMinimumCircleHandlesBoundaryCases) {
  LocalFootstepPlanner planner = makePlanner();

  Eigen::Vector3d circle = planner.welzlMinimumCircle({}, {});
  EXPECT_NEAR(circle.x(), 0.0, kTol);
  EXPECT_NEAR(circle.y(), 0.0, kTol);
  EXPECT_NEAR(circle.z(), 0.0, kTol);

  circle = planner.welzlMinimumCircle({}, {Eigen::Vector2d(1.0, 2.0)});
  EXPECT_NEAR(circle.x(), 1.0, kTol);
  EXPECT_NEAR(circle.y(), 2.0, kTol);
  EXPECT_NEAR(circle.z(), 0.0, kTol);

  circle = planner.welzlMinimumCircle(
      {}, {Eigen::Vector2d(-1.0, 0.0), Eigen::Vector2d(1.0, 0.0)});
  EXPECT_NEAR(circle.x(), 0.0, kTol);
  EXPECT_NEAR(circle.y(), 0.0, kTol);
  EXPECT_NEAR(circle.z(), 1.0, kTol);

  circle = planner.welzlMinimumCircle(
      {}, {Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0),
           Eigen::Vector2d(-1.0, 0.0)});
  EXPECT_NEAR(circle.x(), 0.0, kTol);
  EXPECT_NEAR(circle.y(), 0.0, kTol);
  EXPECT_NEAR(circle.z(), 1.0, kTol);
}

TEST(LocalFootstepPlannerTest, ComputeSwingApexRespectsClearanceBounds) {
  LocalFootstepPlanner planner = makeGo2Planner();
  Eigen::VectorXd body = Eigen::VectorXd::Zero(12);
  body[2] = 0.35;
  const Eigen::Vector3d prev(0.2, 0.1, 0.02);
  const Eigen::Vector3d next(0.35, 0.1, 0.02);

  const double apex = planner.computeSwingApex(0, body, prev, next);

  EXPECT_GE(apex, 0.0);
  EXPECT_LE(apex, body[2]);
  EXPECT_GT(apex, prev.z());
  EXPECT_GT(apex, next.z());
}

TEST(LocalFootstepPlannerTest, ComputeFootPlanHandlesSwingAndTouchdown) {
  LocalFootstepPlanner planner = makeGo2Planner();
  std::vector<std::vector<bool>> schedule = {
      {true, true, true, true},   {false, true, true, true},
      {false, true, true, true},  {true, true, true, true},
      {true, true, true, true},   {true, true, true, true},
  };
  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  Eigen::MatrixXd ref_body_plan = Eigen::MatrixXd::Zero(6, 12);
  for (int i = 0; i < 6; ++i) {
    body_plan(i, 0) = 0.05 * i;
    body_plan(i, 2) = 0.35;
    body_plan(i, 6) = 0.2;
    ref_body_plan.row(i) = body_plan.row(i);
  }
  Eigen::MatrixXd grf_plan = Eigen::MatrixXd::Zero(5, 12);
  Eigen::VectorXd current_feet(12);
  current_feet << 0.20, 0.12, 0.02, 0.20, -0.12, 0.02, -0.20, 0.12, 0.02,
      -0.20, -0.12, 0.02;
  Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(12);
  Eigen::MatrixXd feet = Eigen::MatrixXd::Zero(6, 12);
  Eigen::MatrixXd foot_vel = Eigen::MatrixXd::Zero(6, 12);
  Eigen::MatrixXd foot_acc = Eigen::MatrixXd::Zero(6, 12);
  for (int i = 0; i < 6; ++i) {
    feet.row(i) = current_feet;
  }

  quad_msgs::msg::MultiFootState past;
  past.feet.resize(4);
  past.traj_index = 0;
  for (int foot = 0; foot < 4; ++foot) {
    setFoot(past, foot, current_feet[3 * foot], current_feet[3 * foot + 1],
            current_feet[3 * foot + 2], 0);
  }

  planner.computeFootPlan(0, schedule, body_plan, grf_plan, ref_body_plan,
                          current_feet, current_vel, 0.1, past, feet, foot_vel,
                          foot_acc);

  EXPECT_EQ(past.feet[0].traj_index, 1u);
  EXPECT_NEAR(feet(1, 2), current_feet[2], kTol);
  EXPECT_GT(feet(2, 2), current_feet[2]);
  EXPECT_TRUE(feet(2, 0) > current_feet[0]);
  EXPECT_NEAR(feet(3, 2), 0.02, kTol);
  EXPECT_NEAR(feet(4, 0), feet(3, 0), kTol);
  EXPECT_NEAR(feet(4, 1), feet(3, 1), kTol);
}

// ---- Phase 2A: FootPlanResult (safe-stop signal) ----

// Helper: run the same swing/touchdown scenario as
// ComputeFootPlanHandlesSwingAndTouchdown, optionally after swapping in a
// terrain grid, and return both the FootPlanResult and the foot matrix.
namespace {
struct FootPlanRun {
  FootPlanResult result;
  Eigen::MatrixXd feet;
};

FootPlanRun runFootPlanScenario(LocalFootstepPlanner& planner) {
  std::vector<std::vector<bool>> schedule = {
      {true, true, true, true},   {false, true, true, true},
      {false, true, true, true},  {true, true, true, true},
      {true, true, true, true},   {true, true, true, true},
  };
  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  Eigen::MatrixXd ref_body_plan = Eigen::MatrixXd::Zero(6, 12);
  for (int i = 0; i < 6; ++i) {
    body_plan(i, 0) = 0.05 * i;
    body_plan(i, 2) = 0.35;
    body_plan(i, 6) = 0.2;
    ref_body_plan.row(i) = body_plan.row(i);
  }
  Eigen::MatrixXd grf_plan = Eigen::MatrixXd::Zero(5, 12);
  Eigen::VectorXd current_feet(12);
  current_feet << 0.20, 0.12, 0.02, 0.20, -0.12, 0.02, -0.20, 0.12, 0.02, -0.20,
      -0.12, 0.02;
  Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(12);
  Eigen::MatrixXd feet = Eigen::MatrixXd::Zero(6, 12);
  Eigen::MatrixXd foot_vel = Eigen::MatrixXd::Zero(6, 12);
  Eigen::MatrixXd foot_acc = Eigen::MatrixXd::Zero(6, 12);
  for (int i = 0; i < 6; ++i) {
    feet.row(i) = current_feet;
  }
  quad_msgs::msg::MultiFootState past;
  past.feet.resize(4);
  past.traj_index = 0;
  for (int foot = 0; foot < 4; ++foot) {
    setFoot(past, foot, current_feet[3 * foot], current_feet[3 * foot + 1],
            current_feet[3 * foot + 2], 0);
  }
  FootPlanRun run;
  run.result =
      planner.computeFootPlan(0, schedule, body_plan, grf_plan, ref_body_plan,
                              current_feet, current_vel, 0.1, past, feet,
                              foot_vel, foot_acc);
  run.feet = feet;
  return run;
}
}  // namespace

// On flat traversable terrain every touchdown gets a valid foothold, so the
// plan is safe to hand to NMPC (ok == true, no failures recorded).
TEST(LocalFootstepPlannerTest, ComputeFootPlanReportsOkOnFlatTerrain) {
  LocalFootstepPlanner planner = makeGo2Planner();
  const FootPlanRun run = runFootPlanScenario(planner);

  EXPECT_TRUE(run.result.ok);
  EXPECT_EQ(run.result.failed_count, 0);
  EXPECT_EQ(run.result.worst_status, FootholdStatus::VALID);
  EXPECT_EQ(run.result.failed_leg, -1);
  EXPECT_EQ(run.result.failed_touchdown_index, -1);
}

// When no cell is traversable, the leg-0 touchdown at horizon index 3 cannot be
// placed. computeFootPlan must (a) report ok == false with the first failure's
// details and a count, and (b) NOT write the hole nominal into the plan -- the
// failed touchdown inherits the previous foothold instead.
TEST(LocalFootstepPlannerTest, ComputeFootPlanReportsInvalidOverHole) {
  LocalFootstepPlanner planner = makeGo2Planner();
  const auto blocked_grid = makeTerrain(0.0, 0.0);  // traversability 0 everywhere
  FastTerrainMap blocked_terrain;
  blocked_terrain.loadDataFromGridMap(blocked_grid);
  planner.updateMap(blocked_grid);
  planner.updateMap(blocked_terrain);

  const FootPlanRun run = runFootPlanScenario(planner);

  EXPECT_FALSE(run.result.ok);
  EXPECT_GE(run.result.failed_count, 1);
  EXPECT_EQ(run.result.failed_leg, 0);
  EXPECT_EQ(run.result.failed_touchdown_index, 3);
  EXPECT_EQ(run.result.worst_status, FootholdStatus::NO_TRAVERSABLE_CANDIDATE);
  // The leg-0 touchdown at horizon index 3 inherited the previous foothold
  // (current_feet for leg 0 = {0.20, 0.12, 0.02}) instead of the raibert
  // nominal over the hole (x was ~0.37 in the DIAG log). No NaN leaks through.
  EXPECT_TRUE(run.feet.allFinite());
  EXPECT_NEAR(run.feet(3, 0), 0.20, kTol);
  EXPECT_NEAR(run.feet(3, 1), 0.12, kTol);
  EXPECT_NEAR(run.feet(3, 2), 0.02, kTol);
}

TEST(LocalFootstepPlannerTest, FootPlanMessagesContainTouchdownsAndTimestamps) {
  LocalFootstepPlanner planner = makePlanner();
  std::vector<std::vector<bool>> schedule = {
      {false, true, true, true},
      {true, true, true, true},
      {true, false, true, true},
      {true, true, true, true},
  };
  Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(4, 12);
  Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(4, 12);
  Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(4, 12);
  positions(1, 0) = 0.4;
  positions(3, 3) = -0.2;

  quad_msgs::msg::MultiFootPlanDiscrete footholds;
  quad_msgs::msg::MultiFootPlanContinuous continuous;
  continuous.header.stamp.sec = 10;
  footholds.header = continuous.header;

  planner.loadFootPlanMsgs(schedule, 7, 0.05, positions, velocities,
                           accelerations, footholds, continuous);

  ASSERT_EQ(continuous.states.size(), 4u);
  EXPECT_EQ(continuous.states[0].traj_index, 7);
  EXPECT_EQ(continuous.states[3].traj_index, 10);
  EXPECT_NEAR(
      (rclcpp::Time(continuous.states[1].header.stamp) -
       rclcpp::Time(continuous.states[0].header.stamp))
          .seconds(),
      0.05, kTol);
  ASSERT_EQ(footholds.feet.size(), 4u);
  ASSERT_EQ(footholds.feet[0].footholds.size(), 1u);
  ASSERT_EQ(footholds.feet[1].footholds.size(), 1u);
  EXPECT_NEAR(footholds.feet[0].footholds.front().position.x, 0.4, kTol);
}
