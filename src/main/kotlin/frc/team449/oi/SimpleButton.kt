package frc.team449.oi.buttons

import com.fasterxml.jackson.annotation.JsonCreator
import com.fasterxml.jackson.annotation.JsonIdentityInfo
import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.annotation.ObjectIdGenerators
import edu.wpi.first.wpilibj.GenericHID

/** A version of [JoystickButton] that is a Button.  */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator::class)
class SimpleButton @JsonCreator constructor(
  /** The joystick the button is on.  */
  @param:JsonProperty(required = true) private val joystick: GenericHID,
  /** The port of the button on the joystick.  */
  @param:JsonProperty(required = true) private var buttonNumber: Int
) {
  /**
   * Default constructor.
   *
   * @param joystick The joystick the button is on.
   * @param buttonNumber The port of the button. Note that button numbers begin at 1, not 0.
   */
  init {
    buttonNumber = buttonNumber
  }

  /**
   * Get whether the button is pressed.
   *
   * @return true if the button is pressed, false otherwise.
   */
  fun get(): Boolean {
    return joystick.getRawButton(buttonNumber)
  }
}
