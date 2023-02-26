package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.team449.robot2023.subsystems.arm.control.ArmState

object ArmConstants {

  // Motor CAN ID
  const val PIVOT_MOTOR_ID1 = 5
  const val PIVOT_MOTOR_ID2 = 6
  const val JOINT_MOTOR_ID = 7

  // Encoder constants
  const val PIVOT_ENCODER_CHAN = 8
  const val JOINT_ENCODER_CHAN = 1
  const val PIVOT_ENCODER_OFFSET = 0.357825
  const val JOINT_ENCODER_OFFSET = .171312 // 0.855081

  // PD Controller Constants
  const val kP1 = 9.0
  const val kP2 = 8.0
  const val kD1 = .0
  const val kD2 = .00
  const val kI1 = .001
  const val kI2 = .005
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
    Rotation2d.fromDegrees(17.28),
    0.0,
    0.0
  )
  val MID = ArmState(
    Rotation2d.fromDegrees(62.64),
    Rotation2d.fromDegrees(-52.2),
    0.0,
    0.0
  )
  val LOW = ArmState(
    Rotation2d.fromDegrees(73.39),
    Rotation2d.fromDegrees(-127.10),
    0.0,
    0.0
  )
  val CUBE = ArmState(
    Rotation2d.fromDegrees(62.72),
    Rotation2d.fromDegrees(-126.09),
    0.0,
    0.0
  )
  val STOW = ArmState(
    Rotation2d.fromDegrees(93.27),
    Rotation2d.fromDegrees(-143.07),
    0.0,
    0.0
  )
  val PICKUP = ArmState(
    Rotation2d.fromDegrees(85.64),
    Rotation2d.fromDegrees(-87.31),
    0.0,
    0.0
  )
  val CONE = ArmState(
    Rotation2d.fromDegrees(63.52),
    Rotation2d.fromDegrees(-132.3),
    0.0,
    0.0
  )

  val STATES = listOf(HIGH, MID, LOW, CUBE, STOW, PICKUP, CONE)
}
