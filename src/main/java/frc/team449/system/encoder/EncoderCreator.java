package frc.team449.system.encoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Create an encoder given a motor controller and its configuration
 *
 * @param <M> The type of the motor controller
 */
@FunctionalInterface
public interface EncoderCreator<M extends MotorController> {
  Encoder create(String encName, M motor, boolean inverted);
}
