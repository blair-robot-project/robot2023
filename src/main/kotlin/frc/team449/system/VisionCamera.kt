package frc.team449.system

import edu.wpi.first.math.geometry.*
import org.photonvision.PhotonCamera
import org.photonvision.common.hardware.VisionLEDMode

class VisionCamera(
  cameraName: String,
  private val robotToCamera: Transform3d = Transform3d()
) : PhotonCamera(cameraName) {
  init {
    setLED(VisionLEDMode.kOff)
  }

  /**
   * @param targetPose the pose of the target [ex. Apriltag, reflective tape, etc] on the the field
   * @return the pose of the camera in relative to the field
   */
  fun camPose(targetPose: Pose3d): Pose3d {
    val result = latestResult
    val camToTarget: Transform3d = result.bestTarget.bestCameraToTarget
    val fieldToCam: Pose3d = targetPose.transformBy(camToTarget.inverse())
    return fieldToCam.transformBy(robotToCamera.inverse())
  }

  /**
   * @return if the camera has a target or not
   */
  fun hasTarget(): Boolean {
    return latestResult.hasTargets()
  }

  /**
   * @return the FPGA timestamp (seconds since the robot started running)
   */
  fun timestamp(): Double {
    return latestResult.timestampSeconds
  }
}
