package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.util.Units

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID = 1
  const val JOINT_MOTOR_ID = 2

  // PD Controller Constants
  const val kP1 = .0
  const val kP2 = .0
  const val kD1 = .0
  const val kD2 = .0

  // Length of segments
  val LENGTH_1 = Units.inchesToMeters(12.0)
  val LENGTH_2 = Units.inchesToMeters(5.0)

  // Mass of segments
  const val MASS_1 = 2.3
  const val MASS_2 = .7

  // Gearing of motors
  const val G1 = 1 / 81.0
  const val G2 = 1 / 81.0

  // Distance from pivot to CG for each segment
  val R1 = Units.inchesToMeters(10.5 / 2)
  val R2 = Units.inchesToMeters(11.5)

  // Feedforward constants of first joint in arm
  const val KS1 = 0.0
  const val KV1 = 0.0
  const val KA1 = 0.0
  const val KG1 = 0.0

  // Feedforward constants of second joint in arm
  const val KS2 = 0.0
  const val KV2 = 0.0
  const val KA2 = 0.0
  const val KG2 = 0.0
}
