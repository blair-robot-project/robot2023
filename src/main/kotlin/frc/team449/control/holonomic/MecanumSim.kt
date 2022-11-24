package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax

class MecanumSim(
  ahrs: AHRS,
) : MecanumDrive(
  createSparkMax("frontLeft", DriveConstants.DRIVE_MOTOR_FL, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING)),
  createSparkMax("frontRight", DriveConstants.DRIVE_MOTOR_FR, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING)),
  createSparkMax("backLeft", DriveConstants.DRIVE_MOTOR_BL, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING)),
  createSparkMax("backRight", DriveConstants.DRIVE_MOTOR_BR, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING)),
  Translation2d(DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2),
  Translation2d(DriveConstants.WHEELBASE / 2, -DriveConstants.TRACKWIDTH / 2),
  Translation2d(-DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2),
  Translation2d(-DriveConstants.WHEELBASE / 2, -DriveConstants.TRACKWIDTH / 2),
  ahrs,
  DriveConstants.MAX_LINEAR_SPEED,
  DriveConstants.MAX_ROT_SPEED,
  SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
  { PIDController(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD) }
) {
  private var lastTime = getFPGATimestamp()

  override var heading = Rotation2d(0.0)

  override fun periodic() {
    val currTime = getFPGATimestamp()
    this.heading = this.heading.plus(Rotation2d(this.kinematics.toChassisSpeeds(MecanumDriveWheelSpeeds()).omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime
    ahrs.heading = this.heading
    super.periodic()
  }
}
