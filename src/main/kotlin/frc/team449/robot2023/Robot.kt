package frc.team449.robot2023

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.arm.control.ArmEncoder
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.MotorInversionTest
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.AHRS
import frc.team449.system.motor.createSparkMax

class Robot : RobotBase() {

  val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

//  override val drive = SwerveDrive.swerveDrive(ahrs)
//
//  @Log(name = "Joystick Input")
//  override val oi = createHolonomicOI(drive, driveController)

//  private val pivotMotor = createSparkMax(
//    "Pivot Motor",
//    ArmConstants.PIVOT_MOTOR_ID1,
//    ArmEncoder.creator(
//      ArmConstants.PIVOT_ENCODER_CHAN,
//      ArmConstants.PIVOT_ENCODER_OFFSET,
//      true
//    ),
//    slaveSparks = mapOf(
//      ArmConstants.PIVOT_MOTOR_ID2 to true
//    ),
//    currentLimit = 40,
//    inverted = false
//  )
//
//  private val jointMotor = createSparkMax(
//    "Joint Motor",
//    ArmConstants.JOINT_MOTOR_ID,
//    ArmEncoder.creator(
//      ArmConstants.JOINT_ENCODER_CHAN,
//      ArmConstants.JOINT_ENCODER_OFFSET,
//      inverted = true
//    ),
//    currentLimit = 40
//  )

  private val motor1 = createSparkMax(
    "motor1",
    ArmConstants.PIVOT_MOTOR_ID1,
    ArmEncoder.creator(
      ArmConstants.PIVOT_ENCODER_CHAN,
      ArmConstants.PIVOT_ENCODER_OFFSET,
      true
    ),
    currentLimit = 40,
    inverted = false
  )

  private val motor2 = createSparkMax(
  " Motor 2",
  ArmConstants.PIVOT_MOTOR_ID2,
  ArmEncoder.creator(
    ArmConstants.PIVOT_ENCODER_CHAN,
    ArmConstants.PIVOT_ENCODER_OFFSET,
    true
  ),
  currentLimit = 40,
  inverted = false
  )

  val motors = MotorInversionTest(motor1, motor2)
//  val arm = Arm(
//    pivotMotor,
//    jointMotor,
//    TwoJointArmFeedForward.createFromConstants(),
//    ArmPDController(ArmConstants.kP1, ArmConstants.kP2, ArmConstants.kD1, ArmConstants.kD2, ArmConstants.kI1, ArmConstants.kI2),
//    ArmConstants.LENGTH_1,
//    ArmConstants.LENGTH_2
//  )
}
