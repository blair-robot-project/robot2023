package frc.team449.control.holonomic

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import frc.team449.system.AHRS
import org.photonvision.PhotonPoseEstimator

class SwerveSim(
  modules: List<SwerveModule>,
  ahrs: AHRS,
  maxLinearSpeed: Double,
  maxRotSpeed: Double,
  cameras: List<PhotonPoseEstimator>
) : SwerveDrive(modules, ahrs, maxLinearSpeed, maxRotSpeed, cameras) {

  private var lastTime = getFPGATimestamp()

  override fun periodic() {
    val currTime = getFPGATimestamp()
    heading = heading.plus(Rotation2d((super.desiredSpeeds?.omegaRadiansPerSecond ?: 0.0) * (currTime - lastTime)))
    this.lastTime = currTime

    super.periodic()
  }
}
