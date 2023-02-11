package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.util.Units

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID = 1
  const val JOINT_MOTOR_ID = 2

  // PD Controller Constants
  const val kP1 = 2.0
  const val kP2 = 1.7
  const val kD1 = 0.0 // .1
  const val kD2 = 0.0 // .1
  const val kI1 = .00
  const val kI2 = .00

  // Length of segments
  val LENGTH_1 = Units.inchesToMeters(32.0)
  val LENGTH_2 = Units.inchesToMeters(36.5)

  // Mass of segments
  const val MASS_1 = 2.6
  const val MASS_2 = 3.5 - 2.6

  // Gearing of motors
  const val G1 = 1 / 81.0
  const val G2 = 1 / 81.0

  // Distance from pivot to the Center of Grav for each segment
  val R1 = Units.inchesToMeters(9.97)
  val R2 = Units.inchesToMeters(3.75)

  // Feedforward constants of first joint in arm
  const val KS1 = 0.21992 // 0.21992
  const val KV1 = 0.79172 // 0.79172
  const val KA1 = 0.035016 // 0.035016
  const val KG1 = 0.165 // .31923

  // Feedforward constants of second joint in arm
  const val KS2 = 0.16758 // 0.16758
  const val KV2 = 0.74909 // 0.74909
  const val KA2 = 0.028384 // 0.028384
  const val KG2 = 0.020424 // 0.020424
}
