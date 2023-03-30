package frc.team449.robot2023.constants.subsystem

import kotlin.math.PI

object GroundIntakeConstants {

  const val INTAKE_VOLTAGE = 3.5

  /** Ground Intake Motor Constants */
  const val INTAKE_TOP = 8
  const val INTAKE_BOTTOM = 9
  const val UPR = 2 * PI
  const val GEARING = 1.0 / 3.0
  const val TOP_INVERTED = true
  const val BOTTOM_INVERTED = false
  const val CURRENT_LIM = 20

  const val FWD_CHANNEL = 7
  const val REV_CHANNEL = 0
}
