package frc.team449.system

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.*
import org.photonvision.PhotonCamera
import org.photonvision.common.hardware.VisionLEDMode

class VisionCamera(
  cameraName: String,
  private val robotToCamera: Transform3d = Transform3d(),
  private val tagLayout: AprilTagFieldLayout
) : PhotonCamera(cameraName) {
  init {
    setLED(VisionLEDMode.kOff)
  }

  /**
   * @return the pose of the camera in relative to the field
   */
  fun camPose(): Pose3d {
    val targetPose: Pose3d = tagLayout.getTagPose(latestResult.bestTarget.fiducialId).get()
    val cameraToTarget = latestResult.bestTarget.bestCameraToTarget
    return targetPose.plus(cameraToTarget.inverse()).plus(robotToCamera.inverse())
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
