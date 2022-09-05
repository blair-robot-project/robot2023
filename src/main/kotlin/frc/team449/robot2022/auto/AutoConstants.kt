package frc.team449.robot2022.auto

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile

object AutoConstants {

  /** Driving Config */
  const val MAX_VEL = 2.0
  const val MAX_ACC = 2.0
  private const val MAX_ROTVEL = 2.26
  const val MAX_ROTACC = 2.26

  /** PID gains */
  private const val TRANSLATION_KP = 1.0
  private const val ROTATION_KP = 1.0

  /** PID controllers for each degree of freedom */
  val ROT_CONTROLLER = ProfiledPIDController(
    ROTATION_KP,
    .0,
    .0,
    TrapezoidProfile.Constraints(MAX_ROTVEL, MAX_ROTACC)
  )
  val xController = PIDController(TRANSLATION_KP, .0, .0)
  val yController = PIDController(TRANSLATION_KP, .0, .0)

  /** Controller used to calculate Pose error when following path */
  val controller = HolonomicDriveController(
    xController,
    yController,
    ROT_CONTROLLER
  )
}
