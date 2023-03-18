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
  const val FIRST_ENCODER_OFFSET = 0.25 - (-0.102613)
  const val SECOND_ENCODER_OFFSET = -0.180810 // 0.855081
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
  val LENGTH_2 = Units.inchesToMeters(36.5)

  // Mass of segments
  const val MASS_1 = 6.944561
  const val MASS_2 = 6.062712

  // Gearing of motors
  const val G1 = 1 / 25.0
  const val G2 = 1 / 81.0

  // Distance from pivot to the Center of Grav for each segment
  val R1 = Units.inchesToMeters(9.97)
  val R2 = Units.inchesToMeters(26.0)

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
  val HIGH = ArmState(
    Rotation2d.fromDegrees(127.98),
    Rotation2d.fromDegrees(17.87),
    0.0,
    0.0
  )
  val MID = ArmState(
    Rotation2d.fromDegrees(88.25),
    Rotation2d.fromDegrees(94.25),
    0.0,
    0.0
  )
//  val LOW = ArmState(
//    Rotation2d.fromDegrees(79.56),
//    Rotation2d.fromDegrees(-115.38),
//    0.0,
//    0.0
//  )
//  val CUBE = ArmState(
//    Rotation2d.fromDegrees(60.32),
//    Rotation2d.fromDegrees(-122.55),
//    0.0,
//    0.0
//  )
  val STOW = ArmState(
    Rotation2d.fromDegrees(90.0),
    Rotation2d.fromDegrees(-146.14),
    0.0,
    0.0
  )

  val GROUND = ArmState(
    Rotation2d.fromDegrees(50.67),
    Rotation2d.fromDegrees(-118.55),
    0.0,
    0.0
  )
//  {
//    "q1": 1.66,
//    "q2": -1.66,
  val PICKUP = ArmState(
    Rotation2d(1.66),
    Rotation2d(-1.66),
    0.0,
    0.0
  )
//  val CONE = ArmState(
//    Rotation2d.fromDegrees(61.48),
//    Rotation2d.fromDegrees(-124.85),
//    0.0,
//    0.0
//  )

  val STATES = listOf(HIGH, MID, GROUND, STOW)
}
