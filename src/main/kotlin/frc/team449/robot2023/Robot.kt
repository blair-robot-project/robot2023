package frc.team449.robot2023

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.team449.RobotBase
import frc.team449.control.holonomic.OrthogonalHolonomicOI.Companion.createOrthogonalHolonomicOI
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.arm.ArmSim
import frc.team449.robot2023.subsystems.arm.control.ArmEncoder
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.robot2023.subsystems.endEffector.EndEffector
import frc.team449.robot2023.subsystems.endEffector.EndEffectorConstants
import frc.team449.robot2023.subsystems.groundIntake.GroundIntake
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI

class Robot : RobotBase() {

  val driveController = XboxController(0)

  val mechanismController = XboxController(1)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val drive = SwerveDrive.createSwerve(ahrs)

  @Log(name = "Joystick Input")
  override val oi = createOrthogonalHolonomicOI(drive, driveController)

  private val firstJointMotor = createSparkMax(
    "First Joint Motor",
    ArmConstants.FIRST_MOTOR_ID1,
    ArmEncoder.creator(
      ArmConstants.FIRST_ENCODER_CHAN,
      ArmConstants.FIRST_ENCODER_OFFSET,
      true
    ),
    slaveSparks = mapOf(
      ArmConstants.FIRST_MOTOR_ID2 to true
    ),
    currentLimit = 40,
    inverted = true,
    enableBrakeMode = true
  )

  private val secondJointMotor = createSparkMax(
    "Second Joint Motor",
    ArmConstants.SECOND_MOTOR_ID,
    ArmEncoder.creator(
      ArmConstants.SECOND_ENCODER_CHAN,
      ArmConstants.SECOND_ENCODER_OFFSET,
      inverted = true
    ),
    currentLimit = 40,
    enableBrakeMode = true
  )

  private val firstJointEncoder = QuadEncoder(
    "First joint quad",
    ArmConstants.FIRSTJ_QUAD_ENCODER,
    1024,
    2 * PI,
    1.0
  )

  private val secondJointEncoder = QuadEncoder(
    "Second joint quad",
    ArmConstants.SECONDJ_QUAD_ENCODER,
    1024,
    2 * PI,
    1.0
  )

  val arm = if (isReal())
    Arm(
      firstJointMotor,
      secondJointMotor,
      firstJointEncoder,
      secondJointEncoder,
      TwoJointArmFeedForward.createFromConstants(),
      ArmPDController(
        ArmConstants.kP1,
        ArmConstants.kP2,
        ArmConstants.kD1,
        ArmConstants.kD2,
        ArmConstants.kI1,
        ArmConstants.kI2,
        ArmConstants.kErrDeadband
      ),
      ArmConstants.LENGTH_1,
      ArmConstants.LENGTH_2
    )
  else
    ArmSim(
      firstJointMotor,
      secondJointMotor,
      firstJointEncoder,
      secondJointEncoder,
      TwoJointArmFeedForward.createFromConstants(),
      ArmPDController(
        ArmConstants.kP1,
        ArmConstants.kP2,
        ArmConstants.kD1,
        ArmConstants.kD2,
        ArmConstants.kI1,
        ArmConstants.kI2,
        ArmConstants.kErrDeadband
      ),
      ArmConstants.LENGTH_1,
      ArmConstants.LENGTH_2
    )

  // create intake
  private val intakeClamp = DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    EndEffectorConstants.FORWARD_CHANNEL,
    EndEffectorConstants.REVERSE_CHANNEL
  )

  // create ground intake motors
  private val groundIntakeMotor = createSparkMax(
    "rightGroundIntake",
    20,
    NEOEncoder.creator(
      2 * PI,
      1.0 / 3.0
    ),
    inverted = false,
    currentLimit = 20,
    slaveSparks = mapOf(
      21 to true
    )
  )

  // create ground intake pistons
  private val groundIntakePiston = DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    7,
    0
  )

  private val infrared = DigitalInput(
    EndEffectorConstants.SENSOR_CHANNEL
  )

  @Log(name = "Intake")
  val intake = EndEffector(
    intakeClamp,
    infrared
  )

  val groundIntake = GroundIntake(
    groundIntakeMotor,
    groundIntakePiston,
    arm
  )
}
