package frc.team449.robot2023.subsystems.light

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.LightConstants

class Light(
  private val port: Int,
  private val length: Int
): SubsystemBase() {

  private var lightStrip = AddressableLED(port)
  var buffer = AddressableLEDBuffer(length)

  init {
    lightStrip.setLength(buffer.length)
    lightStrip.setData(buffer)
    lightStrip.start()
  }

  enum class Color(val r: Int, val g: Int, val b: Int) {
    YELLOW(255, 255, 0),
    PURPLE(0, 0, 139),
    RED(255, 0, 0)
  }

  private fun setColor(color: Color) {
    for (i in 0 until buffer.length) {
      buffer.setRGB(i, color.r, color.g, color.b)
    }
  }

  fun setConeColor() {
    setColor(Color.YELLOW)
  }

  fun setCubeColor() {
    setColor(Color.PURPLE)
  }

  fun setDefaultColor() {
    setColor(Color.RED)
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