package frc.team449.robot2023

// import frc.team449.control.holonomic.SwerveDrive.Companion.swerveDrive
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.ArmPDController
import frc.team449.robot2023.subsystems.arm.ArmSim
import frc.team449.robot2023.subsystems.arm.TwoJointArmFeedForward
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI

class Robot : RobotBase() {

  val driveController = XboxController(0)

//  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

//  override val drive = SwerveDrive.swerveDrive(ahrs)
//
//  @Log(name = "Joystick Input")
//  override val oi = createHolonomicOI(drive, driveController)

  private val pivotMotor = createSparkMax(
    "Pivot Motor",
    ArmConstants.PIVOT_MOTOR_ID,
    NEOEncoder.creator(
      2 * PI,
      ArmConstants.G1
    ),
    enableBrakeMode = false
  )

  private val jointMotor = createSparkMax(
    "Joint Motor",
    ArmConstants.JOINT_MOTOR_ID,
    NEOEncoder.creator(
      2 * PI,
      ArmConstants.G2
    ),
    enableBrakeMode = false
  )

  val arm = ArmSim(
    pivotMotor,
    jointMotor,
    TwoJointArmFeedForward.createFromConstants(),
    ArmPDController(ArmConstants.kP1, ArmConstants.kP2, ArmConstants.kD1, ArmConstants.kD2),
    ArmConstants.LENGTH_1,
    ArmConstants.LENGTH_2
  )
}
