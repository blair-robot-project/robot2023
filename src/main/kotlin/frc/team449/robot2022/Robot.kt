package frc.team449.robot2022

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.OIHolonomic.Companion.createHolonomicOI
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveDrive.Companion.swerveDrive
import frc.team449.system.AHRS
import io.github.oblarg.oblog.annotations.Log

class Robot : RobotBase() {

  // Other CAN IDs
  private val PDP_CAN = 1
  private val PCM_MODULE = 0

  private val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  /**
   * Converts the drive to a SwerveSim if the robot is in simulation
   */
  @Log.Include
  override val drive = if (isReal()) swerveDrive(ahrs) else SwerveDrive.simOf(swerveDrive(ahrs))

  override val oi = createHolonomicOI(drive, driveController)
}
