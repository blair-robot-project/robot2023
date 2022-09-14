package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants
import kotlin.math.PI

class HolonomicFollower(
  private val drivetrain: HolonomicDrive,
  private val trajectory: PathPlannerTrajectory,
  private val resetPose: Boolean,
  maxRotVel: Double,
  maxRotAcc: Double
) : CommandBase() {

  private val timer = Timer()
  private var prevTime = 0.0

  private var xController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var yController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var thetaController = ProfiledPIDController(
    AutoConstants.ROTATION_KP, .0, .0,
    TrapezoidProfile.Constraints(maxRotVel, maxRotAcc)
  )
  private val controller = HolonomicDriveController(
    xController, yController, thetaController
  )

  init {
    thetaController.enableContinuousInput(.0, 2 * PI)
  }

  override fun initialize() {
    if (resetPose) {
      drivetrain.pose = trajectory.initialPose
    }

    xController.reset()
    yController.reset()
    thetaController.reset(drivetrain.pose.rotation.radians)
    timer.reset()

    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()
    val reference = trajectory.sample(currTime) as PathPlannerTrajectory.PathPlannerState
    val currentPose = drivetrain.pose

    drivetrain.set(controller.calculate(currentPose, reference.poseMeters, reference.velocityMetersPerSecond, reference.holonomicRotation))

    prevTime = currTime
  }

  override fun isFinished(): Boolean {
    return timer.hasElapsed(trajectory.totalTimeSeconds)
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    drivetrain.stop()
  }
}
