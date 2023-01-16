package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Matrix.mat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N4
import kotlin.math.cos
import kotlin.math.sin

class TwoJointArmFeedForward(
  masses: Pair<Double, Double>,
  lengths: Pair<Double, Double>,
  distanceFromPivot: Pair<Double, Double>,
  momentOfInertia: Pair<Double, Double>,
  gearing: Pair<Double, Double>,
  numMotors: Pair<Double, Double>,
  stallTorque: Double,
  freeSpeed: Double,
  stallCurrent: Double
) {
  private val gravity = 9.8
  private val m1 = masses.first
  private val m2 = masses.second
  private val r1 = distanceFromPivot.first
  private val r2 = distanceFromPivot.second
  private val l1 = lengths.first
  val l2 = lengths.second
  private val i1 = momentOfInertia.first
  private val i2 = momentOfInertia.second
  private val g1 = gearing.first
  private val g2 = gearing.second
  private val n1 = numMotors.first
  private val n2 = numMotors.second
  private val kT = stallTorque / stallCurrent
  private val kV = freeSpeed / 12.0
  private val kR = 12.0 / stallCurrent

  fun calculate(reference: Matrix<N4, N1>, accelReference: Matrix<N2, N1>): Matrix<N2, N1> {
    /** slice the reference state matrix in half*/
    // [theta, beta]
    val angles = reference.block<N2, N1>(2, 1, 0, 0)
    // [theta-dot, beta-dot]
    val angularVelocities = reference.block<N2, N1>(2, 1, 2, 0)
    val theta = angles[0, 0]
    val beta = angles[1, 0]
    val thetaDot = angularVelocities[0, 0]
    val betaDot = angularVelocities[1, 0]

    /** pre-computed cosines and sines*/
    val c1 = cos(theta)
    val c2 = cos(beta)
    val s2 = sin(beta)
    val c12 = cos(theta + beta)

    /** matrix builders for 2x1 and 2x2*/
    val builder2x1 = mat(N2.instance, N1.instance)
    val builder2x2 = mat(N2.instance, N2.instance)

    /** D matrix in equation */
    val D = builder2x2.fill(
      (m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2) + i1 + i2 + 2 * m2 * l1 * r2 * c2), (m2 * r2 * r2 + i2 + m2 * l1 * r2 * c2),
      (m2 * r2 * r2 + i2 + m2 * l1 + r2 * c2), (m2 * r2 * r2 + i2)
    )

    /** C matrix in equation */
    val C = builder2x2.fill(
      (-m2 * l1 * r2 * s2 * betaDot), (-m2 * r2 * r2 + i2 + m2 * l1 * r2 * c2),
      (m2 * l1 * r2 * s2 * thetaDot), (0.0)
    )

    /** Tau g matrix in equation */
    val Tg = builder2x1.fill(
      (gravity * c1) * (m1 * r1 + m2 * l1) + m2 * r2 * gravity * c12,
      m2 * r2 * gravity * c12
    )

    /** Km matrix in equation */
    val Km = builder2x2.fill(
      g1 * n1 * kT / kR, 0.0,
      0.0, g2 * n2 * kT / kR
    )

    /** Kb matrix in equation */
    val Kb = builder2x2.fill(
      (g1 * g1 * n1 * kT) / (kV * kR), 0.0,
      0.0, (g2 * g2 * n2 * kT) / (kV * kR)
    )

    /** Solve equation
     * @see <a href = "https://www.chiefdelphi.com/uploads/short-url/pfucQonJecNeM7gvH57SpOOgPyR.pdf">White paper</a>
     * */
    val dTimesAccel = D.times(accelReference)
    val cTimesVel = C.times(angularVelocities)
    val kbTimesVel = Kb.times(angularVelocities)

    /** return u = Km ^ -1 * [D * accel + C * vel + Tg + Kb * vel] */
    return Km.solve(dTimesAccel.plus(cTimesVel).plus(Tg).plus(kbTimesVel))
  }

  fun calculate(reference: Matrix<N4, N1>): Matrix<N2, N1> {
    // no acceleration
    val acceleration = mat(N2.instance, N1.instance).fill(
      0.0,
      0.0
    )
    return calculate(reference, acceleration)
  }
}
