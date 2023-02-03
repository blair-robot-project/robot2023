package frc.team449.robot2023.constants.arm

import edu.wpi.first.math.util.Units
import kotlin.math.PI

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

  // Distance from pivot to CG for each segment
  val R1 = Units.inchesToMeters(10.5 / 2)
  val R2 = Units.inchesToMeters(11.5)

  // Rotational inertia about center of gravity
  val I1 = R1 * R1 * MASS_1
  val I2 = 1.0 / 12 * MASS_1 * LENGTH_1 * LENGTH_1

  // Gearing of motors
  const val G1 = 1 / 81.0
  const val G2 = 1 / 81.0

  // Number of motors in each gearbox
  const val N1 = 1.0
  const val N2 = 1.0

  // Motor characteristics
  const val STALL_TORQUE = .97
  const val FREE_SPEED = 11000.0 * 2.0 * PI / 60.0
  const val STALL_CURRENT = 100.0
}
