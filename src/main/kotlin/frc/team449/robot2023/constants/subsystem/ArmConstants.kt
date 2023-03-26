package frc.team449.robot2023.constants.subsystem

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import frc.team449.robot2023.subsystems.arm.control.ArmState

object ArmConstants {

  // Motor CAN ID
  const val FIRST_MOTOR_ID1 = 5
  const val FIRST_MOTOR_ID2 = 6
  const val SECOND_MOTOR_ID = 7
  // -
  // 0.430809
  // Encoder constants
  const val FIRST_ENCODER_CHAN = 3
  const val SECOND_ENCODER_CHAN = 0
  const val FIRST_ENCODER_OFFSET = 0.5 - 0.150604
  const val SECOND_ENCODER_OFFSET = -0.185619 // 0.855081
  val FIRSTJ_QUAD_ENCODER = Encoder(4, 5)
  val SECONDJ_QUAD_ENCODER = Encoder(1, 2)
  // PD Controller Constants
  const val kP1 = 10.0
  const val kP2 = 10.0
  const val kD1 = .0
  const val kD2 = .0
  const val kI1 = .1
  const val kI2 = .1
  const val kErrDeadband = .0 // rad
  // Length of segments
  val LENGTH_1 = Units.inchesToMeters(32.0)
  val LENGTH_2 = Units.inchesToMeters(34.0)

  // Mass of segments
  const val MASS_1 = 6.944561
  const val MASS_2 = 6.062712

  // Gearing of motors
  const val G1 = 1 / 25.0
  const val G2 = 1 / 81.0

  // Distance from pivot to the Center of Grav for each segment
  val R1 = Units.inchesToMeters(9.97)
  val R2 = Units.inchesToMeters(24.0)

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

  // Current limits of the motors
  const val FIRST_JOINT_CURR_LIM = 40
  const val SECOND_JOINT_CURR_LIM = 40

  // Arm States corresponding to points.
  // (Joint 1: Rotation2d(Rads: 1.59, Deg: 91.32), Joint 2 : Rotation2d(Rads: -1.94, Deg: -111.19), theta speed : -0.01828647817075757, beta speed 0.0)
  val SINGLE = ArmState(
    Rotation2d.fromDegrees(93.07),
    Rotation2d.fromDegrees(-114.74)
  )

  val DOUBLE = ArmState(
    Rotation2d.fromDegrees(95.11),
    Rotation2d.fromDegrees(-95.11)
  )

  val STOW = ArmState(
    Rotation2d.fromDegrees(90.00),
    Rotation2d.fromDegrees(-151.833816)
  )

  val CONE = ArmState(
    Rotation2d.fromDegrees(47.55),
    Rotation2d.fromDegrees(-116.58)
  )

  val CUBE = ArmState(
    Rotation2d.fromDegrees(54.58),
    Rotation2d.fromDegrees(-120.25)
  )
  val MID = ArmState(
    Rotation2d.fromDegrees(91.40),
    Rotation2d.fromDegrees(90.22)
  )

  val HIGH = ArmState(
    Rotation2d.fromDegrees(133.00),
    Rotation2d.fromDegrees(0.00)
  )

  val STATES = listOf(SINGLE, DOUBLE, STOW, CONE, CUBE, MID, HIGH)
}
