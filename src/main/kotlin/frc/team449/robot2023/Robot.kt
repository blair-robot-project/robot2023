package frc.team449.robot2023

import edu.wpi.first.wpilibj.*
import frc.team449.RobotBase
import frc.team449.control.holonomic.HolonomicOI.Companion.createHolonomicOI
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.ArmSim
import frc.team449.robot2023.subsystems.arm.control.ArmEncoder
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.robot2023.subsystems.intake.Intake
import frc.team449.robot2023.subsystems.intake.IntakeConstants
import frc.team449.system.AHRS
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log

class Robot : RobotBase() {

  val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val drive = SwerveDrive.createSwerve(ahrs)

  @Log(name = "Joystick Input")
  override val oi = createHolonomicOI(drive, driveController)

  private val firstJointMotor = createSparkMax(
    "First Joint Motor",
    ArmConstants.PIVOT_MOTOR_ID1,
    ArmEncoder.creator(
      ArmConstants.PIVOT_ENCODER_CHAN,
      ArmConstants.PIVOT_ENCODER_OFFSET,
      true
    ),
    slaveSparks = mapOf(
      ArmConstants.PIVOT_MOTOR_ID2 to true
    ),
    currentLimit = 40,
    inverted = true,
    enableBrakeMode = true
  )

  private val secondJointMotor = createSparkMax(
    "Second Joint Motor",
    ArmConstants.JOINT_MOTOR_ID,
    ArmEncoder.creator(
      ArmConstants.JOINT_ENCODER_CHAN,
      ArmConstants.JOINT_ENCODER_OFFSET,
      inverted = true
    ),
    currentLimit = 40,
    enableBrakeMode = true
  )

  val arm = ArmSim(
    firstJointMotor,
    secondJointMotor,
    TwoJointArmFeedForward.createFromConstants(),
    ArmPDController(ArmConstants.kP1, ArmConstants.kP2, ArmConstants.kD1, ArmConstants.kD2, ArmConstants.kI1, ArmConstants.kI2),
    ArmConstants.LENGTH_1,
    ArmConstants.LENGTH_2
  )

  // create intake
  private val intakeClamp = DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    IntakeConstants.FORWARD_CHANNEL,
    IntakeConstants.REVERSE_CHANNEL
  )

  private val infrared = DigitalInput(
    IntakeConstants.SENSOR_CHANNEL
  )

  @Log(name = "Intake")
  val intake = Intake(
    intakeClamp,
    infrared
  )
}
