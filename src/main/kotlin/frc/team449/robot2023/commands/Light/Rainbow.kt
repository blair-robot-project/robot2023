package frc.team449.robot2023.commands.Light

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.light.Light

class Rainbow(
  private val led: Light
) : CommandBase() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstHue = 100

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      val hue = (firstHue + i * 180 / led.buffer.length) % 180
      led.buffer.setHSV(i, hue, 255, 128)
    }
    firstHue += 1
    firstHue %= 180
  }
}
