package frc.team449.control.differential

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer
import frc.team449.system.encoder.SimEncoder.Companion.SimEncoderController
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.team449.system.encoder.SimEncoder
import frc.team449.control.DriveSubsystem;
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

class DifferentialSim (private val drivetrain: DifferentialDrivetrainSim,
                       leftEncoder: SimEncoder,
                       rightEncoder: SimEncoder,
                       trackWidth : Double,
                       val maxLinearSpeed : Double) : DriveSubsystem{

  val kinematics = DifferentialDriveKinematics(trackWidth)
  private var desiredSpeeds = DifferentialDriveWheelSpeeds(.0,.0)
  private val leftController = SimEncoderController(leftEncoder)
  private val rightController = SimEncoderController(rightEncoder)
  private var lastTime = Timer.getFPGATimestamp()

  override val heading : Rotation2d
    get(){ return drivetrain.heading}

  override var pose: Pose2d
    get() = drivetrain.pose
    set(pose) {
      drivetrain.pose = pose
    }

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
  }

  override fun stop() {
    drivetrain.setInputs(.0,.0)
  }

  override fun periodic() {
    drivetrain.setInputs(desiredSpeeds.leftMetersPerSecond * 12, desiredSpeeds.rightMetersPerSecond * 12)
    leftController.velocity = drivetrain.leftVelocityMetersPerSecond
    rightController.velocity = drivetrain.rightVelocityMetersPerSecond
    leftController.position = drivetrain.leftPositionMeters
    rightController.position = drivetrain.rightPositionMeters
    val currTime = Timer.getFPGATimestamp()
    // update the drivetrain
    drivetrain.update(currTime - lastTime)
    this.lastTime = currTime
  }
}