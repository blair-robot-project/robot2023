package frc.team449.robot2023

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.OrthogonalHolonomicOI.Companion.createOrthogonalHolonomicOI
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.GroundIntakeConstants
import frc.team449.robot2023.subsystems.arm.Arm.Companion.createArm
import frc.team449.robot2023.subsystems.arm.ArmSim.Companion.createArmSim
import frc.team449.robot2023.subsystems.endEffector.EndEffector.Companion.createEndEffector
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
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

  val arm = if (isReal()) createArm() else createArmSim()

  @Log(name = "End Effector")
  val endEffector = createEndEffector()

  // create ground end effector motors
  private val groundIntakeMotor = createSparkMax(
    "GroundIntake",
    GroundIntakeConstants.INTAKE_RIGHT,
    NEOEncoder.creator(
      2 * PI,
      1.0 / 3.0
    ),
    inverted = true,
    currentLimit = 20,
    slaveSparks = mapOf(
      GroundIntakeConstants.INTAKE_LEFT to true
    )
  )

  // create ground intake pistons
  private val groundIntakePiston = DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    7,
    0
  )

//  val groundIntake = GroundIntake(
//    groundIntakePiston,
//    groundIntakeMotor
//  )
}
