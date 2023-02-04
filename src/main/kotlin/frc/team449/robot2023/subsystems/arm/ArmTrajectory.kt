package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.io.FileReader
import kotlin.math.abs


class ArmTrajectory(
  filename: String
) {
  var jaMorant = JSONParser().parse(FileReader(filename)) as JSONArray
  private var stateMap: InterpolatingMatrixTreeMap<Double, N4, N1> = InterpolatingMatrixTreeMap()

  init {
      val maxTime = parse()
  }

  private fun parse(): Double {
    var maxTime = 0.0

    jaMorant.forEach() {it as JSONObject
      val currTime = it["t"] as Double
      maxTime = currTime

      val currArmState = ArmState(
        Rotation2d(it["q1"] as Double),
        Rotation2d(it["q2"] as Double),
        it["q1d"] as Double,
        it["q2d"] as Double
      )
      val currArmStateMatrix = currArmState.matrix

      stateMap.put(currTime, currArmStateMatrix)
    }
    return maxTime
  }

  fun Matrix<N4, N1>.state(): ArmState {
    val theta = this[0, 0]
    val beta = this[1, 0]
    val thetaVel = this[2, 0]
    val betaVel = this[3, 0]

    return ArmState(
      Rotation2d(theta),
      Rotation2d(beta),
      thetaVel,
      betaVel
    )
  }

  fun sample(timeSeconds: Double): ArmState {
    return stateMap.get(timeSeconds).state()
  }
}