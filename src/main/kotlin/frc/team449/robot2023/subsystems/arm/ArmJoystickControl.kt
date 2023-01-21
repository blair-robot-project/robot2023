package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.wpilibj2.command.CommandBase

class ArmJoystickControl(
  val arm: Arm,
  val x: () -> Double,
  val z: () -> Double
) : CommandBase() {
  init {
    addRequirements(arm)
  }
  override fun execute() {
    val current = arm.coordinate
    val x = x()
    val z = z()
    arm.coordinate = CartesianArmState(
      current.x + x * .02,
      current.z - z * .02,
      x,
      -z
    )
  }
}
