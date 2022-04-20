package frc.team449.robot2022.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class DriveConstants {
  /** Drive motor ports */
  public static final int FRONT_LEFT_DRIVE = 0,
      FRONT_RIGHT_DRIVE = 1,
      BACK_LEFT_DRIVE = 2,
      BACK_RIGHT_DRIVE = 3,
      FRONT_LEFT_TURN = 4,
      FRONT_RIGHT_TURN = 5,
      BACK_LEFT_TURN = 6,
      BACK_RIGHT_TURN = 7;

  /** External encoders for driving motors */
  public static final Encoder FRONT_LEFT_DRIVE_ENC = new Encoder(0, 1),
      FRONT_RIGHT_DRIVE_ENC = new Encoder(2, 3),
      BACK_LEFT_DRIVE_ENC = new Encoder(4, 5),
      BACK_RIGHT_DRIVE_ENC = new Encoder(6, 7);

  public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double DRIVE_GEARING = 5.86;
  public static final double DRIVE_UPR = 0.3021211527151539;
  public static final int NEO_ENCODER_CPR = 1;
  /** CPR of external encoders on driving motors */
  public static final int DRIVE_EXT_ENC_CPR = 256;

  /**
   * Location of the front left module
   */
  public static final Translation2d FRONT_LEFT_LOC = new Translation2d(
      Units.inchesToMeters(25) / 2,
      Units.inchesToMeters(25) / 2);

  public static final int DRIVE_CURRENT_LIM = 40;
  public static final double DRIVE_ENC_VEL_THRESHOLD = 0.1;
  public static final double MAX_LINEAR_SPEED = 4, MAX_ROT_SPEED = 1;
}
