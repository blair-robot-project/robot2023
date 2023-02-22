package frc.team449.control.holonomic

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import frc.team449.system.AHRS
import org.photonvision.PhotonPoseEstimator

class SwerveSim(
  modules: List<SwerveModule>,
  ahrs: AHRS,
  private val ahrsController: AHRS.SimController,
  maxLinearSpeed: Double,
  maxRotSpeed: Double,
  cameras: List<PhotonPoseEstimator>
) : SwerveDrive(modules, ahrs, maxLinearSpeed, maxRotSpeed, cameras) {
  private var lastTime = getFPGATimestamp()

  override var heading = Rotation2d(0.0)

  override fun periodic() {
    val currTime = getFPGATimestamp()

    this.heading = this.heading.plus(Rotation2d(super.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime

    ahrsController.compassHeading = this.heading.radians
    super.periodic()
  }
}
