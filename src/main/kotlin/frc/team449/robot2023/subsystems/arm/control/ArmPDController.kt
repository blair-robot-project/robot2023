package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Matrix.mat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N4
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

class ArmPDController(
  private val kP1: Double,
  private val kP2: Double,
  private val kD1: Double,
  private val kD2: Double,
  private val kI1: Double,
  private val kI2: Double
) {
  private var setpoint: Matrix<N4, N1>? = null
  private var errorSum = mat(N4.instance, N1.instance).fill(0.0, 0.0, 0.0, 0.0)
  /**
   * @param state the current state observed of the system from sensors
   * @param reference the desired state where the system should be
   * @return voltage for joint1 and joint2 to correct for error
   */
  fun calculate(state: Matrix<N4, N1>, reference: Matrix<N4, N1>): Matrix<N2, N1> {
    setpoint = reference
    val err = reference - state
    errorSum = err + errorSum

    val I = mat(N2.instance, N4.instance).fill(
      kI1, 0.0, 0.0, 0.0,
      0.0, kI2, 0.0, 0.0
    )

    val K = mat(N2.instance, N4.instance).fill(
      kP1, 0.0, kD1, 0.0,
      0.0, kP2, 0.0, kD2
    )
    val output = K * err + I * errorSum
    val u1 = output[0, 0]
    val u2 = output[1, 0]
    output[0, 0] = sign(u1) * min(3.0, abs(u1))
    output[1, 0] = sign(u2) * min(3.0, abs(u2))

    return output
  }

  /**
   * calculate using saved setpoint from last calculation
   */
  fun calculate(state: Matrix<N4, N1>): Matrix<N2, N1> {
    // !! to assert that setpoint shouldn't be null
    return calculate(state, setpoint!!)
  }
}