package frc.team449.system

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import org.photonvision.PhotonCamera
import org.photonvision.common.hardware.VisionLEDMode

class VisionCamera(
  cameraName: String,
  private val robotToCamera: Transform3d = Transform3d(),
  private val tagLayout: AprilTagFieldLayout
) : PhotonCamera(cameraName), Loggable {
  init {
    setLED(VisionLEDMode.kOff)
  }

  @Log.ToString
  var tagID = 0

  /**
   * @return the pose of the camera in relative to the field
   */
  fun camPose(): Pose3d {
    tagID = latestResult.bestTarget.fiducialId
    val targetPose: Pose3d = tagLayout.getTagPose(tagID).get()
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
