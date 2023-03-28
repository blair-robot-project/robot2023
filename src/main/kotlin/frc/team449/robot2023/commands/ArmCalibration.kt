package frc.team449.robot2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm

class ArmCalibration(
  private val arm: Arm,
  private val numSamples: Int = 150
): CommandBase() {

  init {
    addRequirements(arm)
  }

  private var firstJointSamples = mutableListOf<Double>()
  private var secondJointSamples = mutableListOf<Double>()
  private var calibrated = false

  override fun initialize() {
    firstJointSamples.removeAll { true }
    secondJointSamples.removeAll { true }
  }

  override fun execute() {
    if (firstJointSamples.size < numSamples) {
      firstJointSamples.add(arm.firstJoint.position)
      secondJointSamples.add(arm.secondJoint.position)
      return
    }
    if (firstJointSamples.size == numSamples && !calibrated) {
      // update encoder reading of quad
      firstJointSamples.sort()
      secondJointSamples.sort()
      val firstJointPos = firstJointSamples[(firstJointSamples.size * .9).toInt()]
      val secondJointPos = secondJointSamples[(secondJointSamples.size * .9).toInt()]
      arm.getFirstJointEncoder().resetPosition(firstJointPos)
      arm.getSecondJointEncoder().resetPosition(secondJointPos)
      calibrated = true
    }
  }

  override fun isFinished(): Boolean {
    return calibrated
  }

  override fun end(interrupted: Boolean) {
    println("***** Finished Calibrating Quadrature reading *****")
  }
}