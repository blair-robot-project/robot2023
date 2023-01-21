package frc.team449.robot2023.constants.arm

import kotlin.math.PI

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID = 9
  const val JOINT_MOTOR_ID = 5

  // PD Controller Constants
  const val kP1 = 2.0
  const val kP2 = 2.0
  const val kD1 = .05
  const val kD2 = .05

  // Length of segments
  const val LENGTH_1 = 46.25 * .0254
  const val LENGTH_2 = 41.80 * .0254

  // Mass of segments
  const val MASS_1 = 9.34 * .4536
  const val MASS_2 = 9.77 * .4536

  // Distance from pivot to CG for each segment
  const val R1 = 21.64 * .0254
  const val R2 = 26.70 * .0254

  // Rotational inertia about center of gravity
  const val I1 = 2957.05 * .0254 * .0254 * .4536
  const val I2 = 2824.70 * .0254 * .0254 * .4536

  // Gearing of motors
  const val G1 = 140.0
  const val G2 = 90.0

  // Number of motors in each gearbox
  const val N1 = 1.0
  const val N2 = 2.0

  // Motor characteristics
  const val STALL_TORQUE = 3.36
  const val FREE_SPEED = 5880.0 * 2.0 * PI / 60.0
  const val STALL_CURRENT = 166.0
}
