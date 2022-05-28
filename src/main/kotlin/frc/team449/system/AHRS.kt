package frc.team449.system

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import frc.team449.util.simBooleanProp
import frc.team449.util.simDoubleProp

class AHRS(port: SerialPort.Port = SerialPort.Port.kMXP) {
  private val navx = com.kauailabs.navx.frc.AHRS(port)
  private var headingOffset = 0.0

  var heading: Rotation2d
    get() {
      return Rotation2d.fromDegrees(headingOffset + this.navx.getFusedHeading())
    }
    set(newHeading) {
      this.headingOffset = newHeading.getDegrees() - this.navx.getFusedHeading()
    }

  /**
   * Used to set properties of an [AHRS] object during simulation. See
   * https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
   *
   * @param devName The name of the simulated device.
   * @param index The NavX index.
   */
  class SimController(devName: String = "navX-Sensor", index: Int = 0) {
    private val deviceSim = SimDeviceSim(devName, index)

    var isConnected by simBooleanProp(deviceSim.getBoolean("Connected"))
    var yaw by simDoubleProp(deviceSim.getDouble("Yaw"))
    var pitch by simDoubleProp(deviceSim.getDouble("Pitch"))
    var roll by simDoubleProp(deviceSim.getDouble("Roll"))
    var compassHeading by simDoubleProp(deviceSim.getDouble("CompassHeading"))
    var fusedHeading by simDoubleProp(deviceSim.getDouble("FusedHeading"))
    var linearWorldAccelX by simDoubleProp(deviceSim.getDouble("LinearWorldAccelX"))
    var linearWorldAccelY by simDoubleProp(deviceSim.getDouble("LinearWorldAccelX"))
    var linearWorldAccelZ by simDoubleProp(deviceSim.getDouble("LinearWorldAccelY"))
  }
}
