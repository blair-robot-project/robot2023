package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.wpilibj.Filesystem
import frc.team449.robot2023.subsystems.arm.Arm
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import java.io.FileWriter

class TrajCharacterizer(
  private val arm: Arm,
  private val trajectory: ArmTrajectory
) : ArmFollower(arm, trajectory) {

  private var prevTime = 0.0
  private var log = JSONArray()

  override fun execute() {
    if (prevTime != 0.0) {
      val currData = JSONObject()
      val currState = arm.state
      val desiredState = arm.desiredState

      currData["t"] = prevTime
      currData["q1"] = currState.theta.radians
      currData["q2"] = currState.beta.radians
      currData["q1-setpoint"] = desiredState.theta.radians
      currData["q2-setpoint"] = desiredState.beta.radians
      currData["u1"] = arm.firstJoint.lastVoltage
      currData["u2"] = arm.secondJoint.lastVoltage

      log.add(currData)
    }
    val currTime = timer.get()
    val reference: ArmState = trajectory.sample(currTime)
    reference.betaVel = 0.0 // want holding data
    reference.thetaVel = 0.0
    arm.state = reference
    prevTime = currTime
  }
  override fun end(interrupted: Boolean) {
    super.end(interrupted)
    val writer = FileWriter("${Filesystem.getDeployDirectory()}/characterization_data.json")
    writer.write(log.toJSONString())
    writer.flush()
    writer.close()
  }
}
