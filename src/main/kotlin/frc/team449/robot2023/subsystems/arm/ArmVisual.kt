package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

/**
 * Gives a solid view of a two-joint arm
 * @param baseToPivot length from the base to the pivot of the arm
 * @param pivotToEnd length from the pivot to the end effector of the arm
 */
class ArmVisual(
  baseToPivot: Double,
  pivotToEnd: Double
) {
  /** This is the instance of the entire mechanism to be sent the dashboard*/
  private val instance = Mechanism2d(
    2 * (baseToPivot + pivotToEnd),
    2 * (baseToPivot + pivotToEnd)
  )

  /** This contains the root/ where the arm is going to start */
  private val arm = instance.getRoot(
    "arm",
    baseToPivot + pivotToEnd,
    baseToPivot + pivotToEnd
  )

  /** A ligament object representing the first segment of the arm*/
  private val firstSegment = arm.append(
    MechanismLigament2d(
      "first segment of arm",
      baseToPivot,
      0.0,
      15.0,
      Color8Bit(Color.kOrange)
    )
  )

  /** A ligament object representing the second segment of the arm */
  private val secondSegment = firstSegment.append(
    MechanismLigament2d(
      "second segment of arm",
      pivotToEnd,
      0.0,
      15.0,
      Color8Bit(Color.kNavy)
    )
  )

  /** send to SmartDashboard to view */
  init {
    instance.setBackgroundColor(Color8Bit(Color.kBlack))
    SmartDashboard.putData("Arm Visual", instance)
  }

  /**
   * Use this method periodically to update the visual to the state of the arm
   * @param state the updated state of the arm to replicate with this visual
   */
  fun setState(state: ArmState) {
    firstSegment.angle = state.theta.degrees
    secondSegment.angle = state.beta.degrees
  }
}
