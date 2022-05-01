package frc.team449.system

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SerialPort

class AHRS(port: SerialPort.Port) {
  private val navx = com.kauailabs.navx.frc.AHRS(port)
  private var headingOffset = 0.0

  var heading: Rotation2d
    get() {
      return Rotation2d.fromDegrees(headingOffset + this.navx.getFusedHeading())
    }
    set(newHeading) {
      this.headingOffset = newHeading.getDegrees() - this.navx.getFusedHeading()
    }
}
