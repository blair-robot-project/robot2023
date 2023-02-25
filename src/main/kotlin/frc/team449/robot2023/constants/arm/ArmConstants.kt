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
  const val JOINT_ENCODER_OFFSET = 1 - 0.854323 // 0.855081

  // PD Controller Constants
  const val kP1 = 11.0
  const val kP2 = 10.0
  const val kD1 = .0
  const val kD2 = .00
  const val kI1 = .000
  const val kI2 = .000

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
    Rotation2d.fromDegrees(128.96),
    Rotation2d.fromDegrees(18.56),
    0.0,
    0.0
  )
  val MID = ArmState(
    Rotation2d.fromDegrees(86.96),
    Rotation2d.fromDegrees(98.80),
    0.0,
    0.0
  )
  val LOW = ArmState(
    Rotation2d.fromDegrees(96.96),
    Rotation2d.fromDegrees(154.15),
    0.0,
    0.0
  )
  val CONE = ArmState(
    Rotation2d.fromDegrees(63.35),
    Rotation2d.fromDegrees(-124.77),
    0.0,
    0.0
  )
  val STOW = ArmState(
    Rotation2d.fromDegrees(93.27),
    Rotation2d.fromDegrees(-144.07),
    0.0,
    0.0
  )

  val STATES = listOf(HIGH, MID, LOW, CONE, STOW)
}
