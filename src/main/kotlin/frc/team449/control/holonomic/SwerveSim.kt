package frc.team449.control.holonomic

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import frc.team449.system.AHRS

class SwerveSim(
  modules: List<SwerveModule>,
  ahrs: AHRS,
  maxLinearSpeed: Double,
  maxRotSpeed: Double
) : SwerveDrive(modules, ahrs, maxLinearSpeed, maxRotSpeed) {
  private var lastTime = getFPGATimestamp()

  override var heading = Rotation2d(0.0)

  override fun periodic() {
    val currTime = getFPGATimestamp()
    this.heading = this.heading.plus(Rotation2d(this.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime
    ahrs.heading = this.heading
    super.periodic()
  }
}
