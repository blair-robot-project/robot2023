package frc.team449.control.holonomic.kinematics

import edu.wpi.first.math.geometry.Translation2d
import org.ejml.simple.SimpleMatrix

class SwerveKinematics(vararg wheelLocations: Translation2d) {
  var numModules = 0
  lateinit var inverseKinematics: SimpleMatrix
  lateinit var forwardKinematics: SimpleMatrix
}
