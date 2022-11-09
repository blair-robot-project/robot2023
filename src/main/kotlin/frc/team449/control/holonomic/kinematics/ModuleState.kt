package frc.team449.control.holonomic.kinematics

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import kotlin.math.abs

class ModuleState(
  theta: Rotation2d = Rotation2d(), // rad
  var omega: Double = 0.0, // rad/s
  speed: Double = 0.0, // m/s
  var accel: Double = 0.0 // m/s^2
) : SwerveModuleState(speed, theta) {

  /**
   When the desired ModuleState is further than 90 degrees from the current module angle,
   optimize such that we pursue the complementary angle with inverted speeds and accelerations.

   Main Purpose: prevent the module from taking long route to reach an angle
   */
  companion object {
    fun optimize(desiredState: ModuleState, currentAngle: Rotation2d): ModuleState {
      val delta = desiredState.angle.minus(currentAngle)
      return if (abs(delta.degrees) > 90.0) {
        ModuleState(
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
          desiredState.omega,
          -desiredState.speedMetersPerSecond,
          -desiredState.accel
        )
      } else {
        desiredState
      }
    }
  }
}
