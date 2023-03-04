package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.team449.robot2023.subsystems.arm.control.ArmState

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID1 = 5
  const val PIVOT_MOTOR_ID2 = 6
  const val JOINT_MOTOR_ID = 7
  // -
  // 0.430809
  // Encoder constants
  const val PIVOT_ENCODER_CHAN = 3
  const val JOINT_ENCODER_CHAN = 1
  const val PIVOT_ENCODER_OFFSET = 0.25 - (-0.102613)
  const val JOINT_ENCODER_OFFSET = -0.180810 // 0.855081

  // PD Controller Constants
  const val kP1 = 9.0
  const val kP2 = 8.0
  const val kD1 = .0
  const val kD2 = .00
  const val kI1 = .001
  const val kI2 = .1
  const val kErrDeadband = .1 // rad
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
  const val KS1 = .0789162
  const val KV1 = 0.54671
  const val KA1 = 1.8353
  const val KG1 = 0.25519

  // Characterized values
  const val KG11 = .161436
  const val KG12 = 0.0
  const val KG21 = 0.0
  const val KG22 = .382553

  // Feedforward constants of second joint in arm
  const val KS2 = .218338
  const val KV2 = 0.59783
  const val KA2 = 3.2914
  const val KG2 = 0.145

  // Arm States corresponding to points.
  val HIGH = ArmState(
    Rotation2d.fromDegrees(126.79),
    Rotation2d.fromDegrees(11.71),
    0.0,
    0.0
  )
  val MID = ArmState(
    Rotation2d.fromDegrees(56.28),
    Rotation2d.fromDegrees(-36.97),
    0.0,
    0.0
  )
  val LOW = ArmState(
    Rotation2d.fromDegrees(67.54),
    Rotation2d.fromDegrees(-115.97),
    0.0,
    0.0
  )
  val CUBE = ArmState(
    Rotation2d.fromDegrees(56.39),
    Rotation2d.fromDegrees(-120.93),
    0.0,
    0.0
  )
  val STOW = ArmState(
    Rotation2d.fromDegrees(96.93),
    Rotation2d.fromDegrees(-136.55),
    0.0,
    0.0
  )
//  val PICKUP = ArmState(
//    Rotation2d.fromDegrees(85.64),
//    Rotation2d.fromDegrees(-87.31),
//    0.0,
//    0.0
//  )
  val CONE = ArmState(
    Rotation2d.fromDegrees(62.62),
    Rotation2d.fromDegrees(-124.44),
    0.0,
    0.0
  )

  val STATES = listOf(HIGH, MID, LOW, CUBE, STOW, CONE)
}
