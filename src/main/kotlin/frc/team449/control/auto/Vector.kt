package frc.team449.control.auto

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

class Vector3d(val x: Double, val y: Double, val z: Double) {
  fun toPose(): Pose2d {
    return Pose2d(Translation2d(x, y), Rotation2d(z))
  }

  operator fun plus(vector: Vector3d): Vector3d {
    return Vector3d(x + vector.x, y + vector.y, z + vector.z)
  }

  operator fun minus(vector: Vector3d): Vector3d {
    return Vector3d(x - vector.x, y - vector.y, z - vector.z)
  }

  operator fun times(scale: Double): Vector3d {
    return Vector3d(x * scale, y * scale, z * scale)
  }

  companion object {
    fun fromPose(pose: Pose2d): Vector3d {
      return Vector3d(pose.x, pose.y, pose.rotation.radians)
    }
  }
}
