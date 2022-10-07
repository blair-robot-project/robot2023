package frc.team449.system

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import frc.team449.util.simBooleanProp
import frc.team449.util.simDoubleProp
import io.github.oblarg.oblog.annotations.Log

class AHRS(private val navx: com.kauailabs.navx.frc.AHRS) : Gyro by navx {
  private var headingOffset = 0.0

  var heading: Rotation2d
    @Log
    get() {
      return Rotation2d.fromDegrees(headingOffset + this.navx.fusedHeading)
    }
    set(newHeading) {
      this.headingOffset = this.navx.fusedHeading - newHeading.degrees
    }

  constructor(port: SerialPort.Port = SerialPort.Port.kMXP) : this(com.kauailabs.navx.frc.AHRS(port))

  override fun reset() {
    navx.zeroYaw()
    heading = Rotation2d()
  }

  override fun getAngle() = heading.degrees

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
