package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import java.io.FileWriter

class TrajCharacterizer(
  private val arm: Arm,
  private val trajectory: ArmTrajectory,
  states: Int,
  private val holdTimeSeconds: Double
) : CommandBase() {

  private val dt = trajectory.totalTime / states.toDouble()
  private var log = JSONArray()
  private var currTime = 0.0
  private val holdTimer = Timer()

  override fun initialize() {
    holdTimer.start()
  }

  override fun execute() {
    val currState = arm.state
    val desiredState = arm.desiredState

    if (currTime != 0.0) {
      val currData = JSONObject()

      currData["t"] = currTime
      currData["q1"] = currState.theta.radians
      currData["q2"] = currState.beta.radians
      currData["q1-setpoint"] = desiredState.theta.radians
      currData["q2-setpoint"] = desiredState.beta.radians
      currData["u1"] = arm.firstJoint.lastVoltage
      currData["u2"] = arm.secondJoint.lastVoltage

      log.add(currData)
    }
    val reference: ArmState = trajectory.sample(currTime)
    reference.betaVel = 0.0 // want holding data
    reference.thetaVel = 0.0
    arm.state = reference
    if (holdTimer.hasElapsed(holdTimeSeconds)) {
      currTime += dt
      holdTimer.reset()
      holdTimer.start()
    }
  }

  override fun isFinished(): Boolean {
    return currTime > trajectory.totalTime
  }
  override fun end(interrupted: Boolean) {
    val writer = FileWriter("${Filesystem.getDeployDirectory()}/characterization_data.json")
    writer.write(log.toJSONString())
    writer.flush()
    writer.close()
    holdTimer.stop()
    holdTimer.reset()
  }
}
