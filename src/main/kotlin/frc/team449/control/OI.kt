package frc.team449.control

import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * An Operator Input (OI) that gets the desired ChassisSpeeds to give a drivetrain
 */

// TEDDY IS A FOO
fun interface OI {
  fun get(): ChassisSpeeds
}
