package frc.team449.system

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import frc.team449.util.simBooleanProp
import frc.team449.util.simDoubleProp
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

class AHRS(private val navx: com.kauailabs.navx.frc.AHRS) : Gyro by navx, Loggable {

  /** The current reading of the gyro with the offset included */
  @get:Log.ToString
  val heading: Rotation2d
    get() {
      return -Rotation2d.fromDegrees(navx.fusedHeading.toDouble())
    }
  @get:Log.ToString
  val pitch: Rotation2d
    get() {
      return -Rotation2d.fromDegrees(navx.pitch.toDouble())
    }
  @get:Log.ToString
  val roll: Rotation2d
    get() {
      return -Rotation2d.fromDegrees(navx.roll.toDouble())
    }

  constructor(
    port: SerialPort.Port = SerialPort.Port.kMXP
  ) : this(
    com.kauailabs.navx.frc.AHRS(port)
  )

  fun calibrated(): Boolean {
    return navx.isMagnetometerCalibrated
  }

  override fun getAngle() = heading.degrees

  /**
   * Used to set properties of an [AHRS] object during simulation. See
   * https://pdocs.kauailabs.com/navx-mxp/softwa
   * re/roborio-libraries/java/
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
