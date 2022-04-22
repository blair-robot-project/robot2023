package frc.team449.robot2022

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.holonomic.OIHolonomic
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.BackupEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log

class RobotContainer2022 {

  // Other CAN IDs
  val PDP_CAN = 1
  val PCM_MODULE = 0

  val driveController = XboxController(0)

  val ahrs = AHRS(SerialPort.Port.kMXP)

  val field = Field2d()
  val autoChooser = SendableChooser<AutoRoutine>()
  // Instantiate/declare PDP and other stuff here

  @Log.Include
  val drive = createDrivetrain()

  val oi = OIHolonomic(
    drive,
    driveController::getLeftY,
    driveController::getLeftX,
    { driveController.getRawAxis(3) },
    SlewRateLimiter(0.5),
    1.5,
    true
  )

  /** Helper to make turning motors for swerve */
  private fun makeDrivingMotor(
    name: String,
    motorId: Int,
    inverted: Boolean,
    wpiEnc: Encoder
  ) =
    createSparkMax(
      name = name + "Drive",
      id = motorId,
      enableBrakeMode = true,
      inverted = inverted,
      encCreator =
      BackupEncoder.creator(
        QuadEncoder.creator(
          wpiEnc,
          DriveConstants.DRIVE_EXT_ENC_CPR,
          DriveConstants.DRIVE_UPR,
          1.0
        ),
        NEOEncoder.creator(
          DriveConstants.DRIVE_UPR,
          DriveConstants.DRIVE_GEARING
        ),
        DriveConstants.DRIVE_ENC_VEL_THRESHOLD
      )
    )

  /** Helper to make turning motors for swerve */
  private fun makeTurningMotor(
    name: String,
    motorId: Int,
    inverted: Boolean,
    encoderChannel: Int,
    offset: Double
  ) =
    createSparkMax(
      name = name + "Turn",
      id = motorId,
      enableBrakeMode = true,
      inverted = inverted,
      encCreator = AbsoluteEncoder.creator(
        encoderChannel,
        2 * Math.PI,
        offset,
        DriveConstants.TURN_UPR,
        DriveConstants.TURN_GEARING
      )
    )

  private fun createDrivetrain() =
    SwerveDrive.squareDrive(
      ahrs,
      DriveConstants.MAX_LINEAR_SPEED,
      DriveConstants.MAX_ROT_SPEED,
      makeDrivingMotor(
        "FL",
        DriveConstants.DRIVE_MOTOR_FL,
        false,
        DriveConstants.DRIVE_ENC_FL
      ),
      makeDrivingMotor(
        "FR",
        DriveConstants.DRIVE_MOTOR_FR,
        false,
        DriveConstants.DRIVE_ENC_FR
      ),
      makeDrivingMotor(
        "BL",
        DriveConstants.DRIVE_MOTOR_BL,
        false,
        DriveConstants.DRIVE_ENC_BL
      ),
      makeDrivingMotor(
        "BR",
        DriveConstants.DRIVE_MOTOR_BR,
        false,
        DriveConstants.DRIVE_ENC_BR
      ),
      makeTurningMotor(
        "FL",
        DriveConstants.TURN_MOTOR_FL,
        false,
        DriveConstants.TURN_ENC_CHAN_FL,
        DriveConstants.TURN_ENC_OFFSET_FL
      ),
      makeTurningMotor(
        "FR",
        DriveConstants.TURN_MOTOR_FR,
        false,
        DriveConstants.TURN_ENC_CHAN_FR,
        DriveConstants.TURN_ENC_OFFSET_FR
      ),
      makeTurningMotor(
        "BL",
        DriveConstants.TURN_MOTOR_BL,
        false,
        DriveConstants.TURN_ENC_CHAN_BL,
        DriveConstants.TURN_ENC_OFFSET_BL
      ),
      makeTurningMotor(
        "BR",
        DriveConstants.TURN_MOTOR_BR,
        false,
        DriveConstants.TURN_ENC_CHAN_BR,
        DriveConstants.TURN_ENC_OFFSET_BR
      ),
      DriveConstants.FRONT_LEFT_LOC,
      { PIDController(.0, .0, .0) },
      {
        ProfiledPIDController(
          .0,
          .0,
          .0,
          TrapezoidProfile.Constraints(.0, .0)
        )
      },
      SimpleMotorFeedforward(.0, .0, .0),
      SimpleMotorFeedforward(.0, .0, .0)
    )

  fun teleopInit() {
    // todo Add button bindings here
  }

  fun robotPeriodic() {}

  fun simulationInit() {
    // DriverStationSim.setEnabled(true)
  }

  fun simulationPeriodic() {
    // Update simulated mechanisms on Mechanism2d widget and stuff
  }
}
