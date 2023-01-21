package frc.team449.robot2023

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.MecanumDrive
import frc.team449.control.holonomic.OIHolonomic.Companion.createHolonomicOI
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.*
// import frc.team449.control.holonomic.SwerveDrive.Companion.swerveDrive
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI

class Robot : RobotBase() {

  val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val drive = MecanumDrive.createMecanum(ahrs)

  @Log(name = "Joystick Input")
  override val oi = createHolonomicOI(drive, driveController)

  private val pivotMotor = createSparkMax(
    "Pivot Motor",
    ArmConstants.PIVOT_MOTOR_ID,
    NEOEncoder.creator(
      2 * PI,
      ArmConstants.G1
    )
  )

  private val jointMotor = createSparkMax(
    "Joint Motor",
    ArmConstants.JOINT_MOTOR_ID,
    NEOEncoder.creator(
      2 * PI,
      ArmConstants.G2
    )
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
