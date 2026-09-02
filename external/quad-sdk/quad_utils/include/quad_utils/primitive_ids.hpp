#ifndef QUAD_UTILS_PRIMITIVE_IDS_H
#define QUAD_UTILS_PRIMITIVE_IDS_H

namespace quad_utils {

/**
 * @brief Single source of truth for body-plan primitive ids.
 *
 * These values are carried across process boundaries as `uint32[]
 * primitive_ids` in quad_msgs/RobotPlan and quad_msgs/BodyPlan, and are
 * switched on in the global body planner, the local footstep planner and
 * rviz_interface. Historically the same 0..3 constants were re-declared as
 * literals in three different headers (planning_utils.hpp, the enum `Phase`;
 * local_footstep_planner.hpp; rviz_interface.hpp). They are now all defined
 * here and only referenced elsewhere.
 *
 * IDs 0..3 are the upstream Quad-SDK phases and MUST keep their values.
 * IDs 4..7 are the MPC_DOG Step 17 forward-jump sub-phases; they are only
 * ever emitted when a jump action is planned, so nominal walking / STAND is
 * unaffected. Planned contact schedule per sub-phase (leg order
 * 0=FL 1=RL 2=FR 3=RR):
 *
 *   PRELOAD    {1,1,1,1}  drop hips, shift CoM rearward
 *   REAR_PUSH  {0,1,0,1}  rear legs only, forward-up takeoff impulse
 *   FLIGHT     {0,0,0,0}  ballistic flight
 *   FRONT_LAND {1,0,1,0}  front legs touch down first
 *   SETTLE     {1,1,1,1}  rear legs land, stabilize
 */
enum PrimitiveId {
  PRIM_CONNECT = 0,      //!< normal-locomotion connect stance
  PRIM_LEAP_STANCE = 1,  //!< legacy lumped leap take-off stance
  PRIM_FLIGHT = 2,       //!< ballistic flight, no contact
  PRIM_LAND_STANCE = 3,  //!< legacy lumped landing stance
  PRIM_PRELOAD = 4,      //!< jump: 4-leg stance, drop hips + shift CoM rearward
  PRIM_REAR_PUSH = 5,    //!< jump: rear legs only, forward-up takeoff impulse
  PRIM_FRONT_LAND = 6,   //!< jump: front legs only, first ground contact
  PRIM_SETTLE = 7,       //!< jump: 4-leg stance, rear legs land + stabilize
};

/// @brief True when the primitive id is one of the Step 17 jump sub-phases.
inline bool isJumpPrimitive(int id) {
  return id == PRIM_PRELOAD || id == PRIM_REAR_PUSH || id == PRIM_FRONT_LAND ||
         id == PRIM_SETTLE;
}

/// @brief True when the primitive id implies at least one foot in contact.
inline bool primitiveHasContact(int id) { return id != PRIM_FLIGHT; }

}  // namespace quad_utils

#endif  // QUAD_UTILS_PRIMITIVE_IDS_H
