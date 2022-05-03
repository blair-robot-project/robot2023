package frc.team449.robot2022

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.differential.DifferentialDrive
import frc.team449.control.differential.DifferentialOIs
import frc.team449.control.differential.DifferentialSim
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.BackupEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.encoder.SimEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.absoluteValue

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

  val oi = DifferentialOIs.createCurvature(
    drive,
    {driveController.rightTriggerAxis - driveController.leftTriggerAxis},
    {if (driveController.leftX.absoluteValue < DriveConstants.DRIVE_TURNING_DEADBAND) .0 else driveController.leftX},
    SlewRateLimiter(DriveConstants.LINEAR_ACC_LIMIT),
    SlewRateLimiter(DriveConstants.TURNING_ACC_LIMIT),
    {true}
  )
  /** Helper to make turning motors for swerve */
  private fun makeSide(
    name: String,
    motorId: Int,
    inverted: Boolean,
    wpiEnc: Encoder,
    followers: Map<Int, Boolean>
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
      ),
      slaveSparks = followers
    )
  private fun createDrivetrain() =
//      if (RobotBase.isReal())
        DifferentialDrive(
        leftLeader = makeSide(
          "Left_",
          DriveConstants.DRIVE_MOTOR_L,
          false,
          DriveConstants.DRIVE_ENC_LEFT,
          mapOf( Pair(DriveConstants.DRIVE_MOTOR_L1, false),
            Pair(DriveConstants.DRIVE_MOTOR_L2, false))
        ),
        rightLeader = makeSide(
          "Right_",
          DriveConstants.DRIVE_MOTOR_R,
          true,
          DriveConstants.DRIVE_ENC_LEFT,
          mapOf( Pair(DriveConstants.DRIVE_MOTOR_R1, false),
            Pair(DriveConstants.DRIVE_MOTOR_R2, false))
        ),
        ahrs,
        SimpleMotorFeedforward(DriveConstants.DRIVE_FF_KS,DriveConstants.DRIVE_FF_KV,DriveConstants.DRIVE_FF_KA),
        { PIDController(DriveConstants.DRIVE_KP_VEL,DriveConstants.DRIVE_KI_VEL,DriveConstants.DRIVE_KD_VEL) },
        DriveConstants.TRACK_WIDTH,
        DriveConstants.MAX_LINEAR_SPEED)
//      /** When in sim */
//      else
//        DifferentialSim(
//          DifferentialDrivetrainSim(
//            LinearSystemId.identifyDrivetrainSystem(
//              DriveConstants.DRIVE_FF_KV, DriveConstants.DRIVE_FF_KA, DriveConstants.DRIVE_ANGLE_FF_KV, DriveConstants.DRIVE_ANGLE_FF_KA
//            ),
//            DCMotor.getNEO(3),
//            DriveConstants.DRIVE_GEARING,
//            DriveConstants.TRACK_WIDTH,
//            DriveConstants.DRIVE_WHEEL_RADIUS,
//            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
//          ),
//          SimEncoder("left_sim_encoder"),
//          SimEncoder("right_sim_encoder"),
//          DriveConstants.TRACK_WIDTH,
//          DriveConstants.MAX_LINEAR_SPEED
//        )

  fun teleopInit() {
    // todo Add button bindings here
  }

  fun robotPeriodic() {

  }

  fun simulationInit() {
    // DriverStationSim.setEnabled(true)
  }

  fun simulationPeriodic() {
    // Update simulated mechanisms on Mechanism2d widget and stuff
  }
}
