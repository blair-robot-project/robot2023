package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.util.Units

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID = 5
  const val PIVOT_MOTOR_ID_2 = 6
  const val JOINT_MOTOR_ID = 7

  // PD Controller Constants
  const val kP1 = 0.0
  const val kP2 = 0.0
  const val kD1 = 0.0 // .1
  const val kD2 = 0.0 // .1
  const val kI1 = 0.0
  const val kI2 = 0.0

  // Length of segments
  val LENGTH_1 = Units.inchesToMeters(37.5)
  val LENGTH_2 = Units.inchesToMeters(34.0)

  // Mass of segments
  const val MASS_1 = 6.944561
  const val MASS_2 = 6.062712

  // Gearing of motors, second values might be inverted??
  const val G1 = (1.0 / 25.0) * (20.0 / 64.0) * (1.0 / 2.0)
  const val G2 = (1.0 / 125.0) * (22.0 / 26.0)

  // Distance from pivot to the Center of Grav for each segment
  val R1 = Units.inchesToMeters(9.97)
  val R2 = Units.inchesToMeters(25.0)

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
