package frc.team449.robot2023.subsystems.light

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.LightConstants

class Light(
  port: Int,
  length: Int
) : SubsystemBase() {

  private var lightStrip = AddressableLED(port)
  private var buffer = AddressableLEDBuffer(length)

  init {
    lightStrip.setLength(buffer.length)
    lightStrip.setData(buffer)
    lightStrip.start()
  }

  private fun setColor(color: Color) {
    for (i in 0 until buffer.length) {
      buffer.setRGB(i, color.red.toInt(), color.green.toInt(), color.blue.toInt())
    }
  }

  fun setConeColor() {
    setColor(kYellow)
  }

  fun setCubeColor() {
    setColor(kPurple)
  }

  fun setDefaultColor() {
    setColor(kRed)
  }

  override fun periodic() {
    lightStrip.setData(buffer)
  }

  companion object {
    fun createLight(): Light {
      return Light(
        LightConstants.LIGHT_PORT,
        LightConstants.LIGHT_LENGTH
      )
    }
  }
}
