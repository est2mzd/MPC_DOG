#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "global_body_planner/gbpl.hpp"
#include "global_body_planner/global_body_plan.hpp"
#include "global_body_planner/global_body_planner_test_fixture.hpp"

namespace {

constexpr double kTol = 1e-6;

planning_utils::State makeState(double x, double y, double z, double vx,
                                double vy, double vz) {
  planning_utils::State s;
  s.pos << x, y, z;
  s.vel << vx, vy, vz;
  return s;
}

}  // namespace

TEST(GlobalBodyPlannerPlanTest, ClearAndInvalidateResetPlanState) {
  GlobalBodyPlan plan;

  EXPECT_TRUE(plan.isEmpty());
  EXPECT_EQ(plan.getStatus(), UNSOLVED);

  plan.invalidate();

  EXPECT_EQ(plan.getStatus(), UNSOLVED);
  EXPECT_DOUBLE_EQ(plan.getLength(), std::numeric_limits<double>::max());

  plan.clear();

  EXPECT_TRUE(plan.isEmpty());
  EXPECT_EQ(plan.getStatus(), UNSOLVED);
  EXPECT_DOUBLE_EQ(plan.getLength(), 0.0);
}

TEST_F(GlobalBodyPlannerTestFixture, LoadPlanDataAndConvertToMsgs) {
  planning_utils::State s1 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s2 = s1;
  s2.pos[0] += 1.0;

  planning_utils::StateActionResult result;
  GBPL gbpl;
  ASSERT_EQ(gbpl.attemptConnect(s1, s2, 2.0, result, planner_config_, FORWARD),
            REACHED);

  std::vector<planning_utils::State> states{s1, s2};
  std::vector<planning_utils::Action> actions{result.a_new};
  planning_utils::FullState start_state =
      planning_utils::stateToFullState(s1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  GlobalBodyPlan plan;
  plan.loadPlanData(VALID, start_state, 0.0, states, actions, 0.03, 0.0,
                    planner_config_);

  EXPECT_EQ(plan.getStatus(), VALID);
  EXPECT_GT(plan.getSize(), 0);
  EXPECT_NEAR(plan.getLength(), 1.0, kTol);
  EXPECT_DOUBLE_EQ(plan.getGoalDistance(), 0.0);

  for (int i = 1; i < plan.getSize(); ++i) {
    EXPECT_GE(plan.getTime(i), plan.getTime(i - 1));
    EXPECT_GE(plan.getLengthAtIndex(i), plan.getLengthAtIndex(i - 1));
  }

  quad_msgs::msg::RobotPlan robot_plan_msg;
  quad_msgs::msg::RobotPlan discrete_plan_msg;
  robot_plan_msg.header.frame_id = "map";
  discrete_plan_msg.header.frame_id = "map";
  plan.convertToMsg(robot_plan_msg, discrete_plan_msg);

  EXPECT_EQ(robot_plan_msg.states.size(), robot_plan_msg.grfs.size());
  EXPECT_EQ(robot_plan_msg.states.size(), robot_plan_msg.plan_indices.size());
  EXPECT_EQ(robot_plan_msg.states.size(), robot_plan_msg.primitive_ids.size());
  EXPECT_EQ(discrete_plan_msg.states.size(), states.size());
}

// Step 17: an Action flagged is_jump is interpolated into the explicit
// PRELOAD -> REAR_PUSH -> FLIGHT -> FRONT_LAND -> SETTLE sub-phase sequence.
// A plain leap (is_jump=false) still reports LEAP_STANCE / FLIGHT /
// LAND_STANCE, so nothing changes for the upstream path.
TEST_F(GlobalBodyPlannerTestFixture, JumpActionInterpEmitsSubPhases) {
  using namespace planning_utils;
  updateTerrainHeight(0.0);

  planning_utils::State s = makeState(0.0, 0.0, 0.3, 0.6, 0.0, 0.0);

  Action a;
  a.grf_0 << 0.0, 0.0, 1.0;
  a.grf_f << 0.0, 0.0, 1.0;
  a.t_s_leap = 0.2;
  a.t_f = 0.3;
  a.t_s_land = 0.2;
  a.dz_0 = 0.0;
  a.dz_f = 0.0;

  planner_config_.jump_preload_fraction = 0.4;
  planner_config_.jump_front_land_fraction = 0.5;

  auto collect = [&](const Action& act) {
    std::vector<planning_utils::State> plan;
    std::vector<planning_utils::GRF> grf;
    std::vector<double> t;
    std::vector<double> len{0.0};  // callers seed the cumulative length with 0
    std::vector<int> ids;
    interpStateActionPair(s, act, 0.0, 0.03, plan, grf, t, ids, len,
                          planner_config_);
    return ids;
  };

  // Plain leap: only legacy ids.
  a.is_jump = false;
  std::vector<int> plain = collect(a);
  for (int id : plain) {
    EXPECT_TRUE(id == LEAP_STANCE || id == FLIGHT || id == LAND_STANCE);
  }

  // Jump: the five sub-phases, in canonical order, no legacy leap/land ids.
  a.is_jump = true;
  std::vector<int> jump = collect(a);
  ASSERT_FALSE(jump.empty());
  EXPECT_EQ(jump.front(), PRELOAD);
  EXPECT_EQ(jump.back(), SETTLE);

  int rank_prev = -1;
  const auto rank = [](int id) {
    switch (id) {
      case PRELOAD: return 0;
      case REAR_PUSH: return 1;
      case FLIGHT: return 2;
      case FRONT_LAND: return 3;
      case SETTLE: return 4;
      default: return 99;
    }
  };
  bool saw_preload = false, saw_rear_push = false, saw_flight = false,
       saw_front_land = false, saw_settle = false;
  for (int id : jump) {
    EXPECT_NE(id, LEAP_STANCE);
    EXPECT_NE(id, LAND_STANCE);
    int r = rank(id);
    ASSERT_LT(r, 99) << "unexpected primitive id " << id;
    EXPECT_GE(r, rank_prev) << "sub-phases went backwards";
    rank_prev = r;
    saw_preload |= (id == PRELOAD);
    saw_rear_push |= (id == REAR_PUSH);
    saw_flight |= (id == FLIGHT);
    saw_front_land |= (id == FRONT_LAND);
    saw_settle |= (id == SETTLE);
  }
  EXPECT_TRUE(saw_preload && saw_rear_push && saw_flight && saw_front_land &&
              saw_settle);
}
