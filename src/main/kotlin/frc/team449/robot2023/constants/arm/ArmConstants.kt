package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.util.Units

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID1 = 5
  const val PIVOT_MOTOR_ID2 = 6
  const val JOINT_MOTOR_ID = 7

  // Encoder constants
  const val PIVOT_ENCODER_CHAN = 8
  const val JOINT_ENCODER_CHAN = 1
  const val PIVOT_ENCODER_OFFSET = 0.356848
  const val JOINT_ENCODER_OFFSET = 1 - 0.854323 // 0.855081

  // PD Controller Constants
  const val kP1 = .00
  const val kP2 = .01
  const val kD1 = .00
  const val kD2 = .00
  const val kI1 = .00
  const val kI2 = .00

  // Length of segments
  val LENGTH_1 = Units.inchesToMeters(37.5)
  val LENGTH_2 = Units.inchesToMeters(34.0)

  // Mass of segments
  const val MASS_1 = 6.944561
  const val MASS_2 = 6.062712

  // Gearing of motors
  const val G1 = 1 / 25.0
  const val G2 = 1 / 81.0

  // Distance from pivot to the Center of Grav for each segment
  val R1 = Units.inchesToMeters(9.97)
  val R2 = Units.inchesToMeters(25.0)

  // Feedforward constants of first joint in arm
  const val KS1 = 1.7589
  const val KV1 = 2.128
  const val KA1 = 3.200
  const val KG1 = 4.000

  // Feedforward constants of second joint in arm
  const val KS2 = 0.4588
  const val KV2 = 0.59783
  const val KA2 = 3.2914
  const val KG2 = 0.46
}
