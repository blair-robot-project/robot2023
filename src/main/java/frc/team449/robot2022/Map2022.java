package frc.team449.robot2022;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.Encoder;
import frc.team449.robot2022.drive.DriveConstants;
import frc.team449.system.encoder.BackupEncoder;
import frc.team449.system.encoder.NEOEncoder;
import frc.team449.system.encoder.QuadEncoder;
import frc.team449.system.motor.SparkMaxConfig;
import frc.team449.util.FollowerUtils;

public final class Map2022 {
  // Other CAN IDs
  public static final int PDP_CAN = 1, PCM_MODULE = 0;
  // Controller ports
  public static final int CARGO_JOYSTICK_PORT = 0,
      DRIVE_JOYSTICK_PORT = 1,
      CLIMBER_JOYSTICK_PORT = 2;
  // Limelight
  public static final int DRIVER_PIPELINE = 0, BLUE_PIPELINE = 1, RED_PIPELINE = 2;
  /** Control loop time */
  public static final double LOOP_TIME = 0.02;

  private Map2022() {}

  @NotNull
  public static void createRobotMap() {
    var leftDriveMaster = new SparkMaxConfig()
        .setPort(DriveConstants.LEFT_LEADER_PORT)
        .setName("DriveLeftMaster")
        .setEnableBrakeMode(true)
        .setInverted(false)
        .addSlaveSpark(FollowerUtils.createFollowerSpark(DriveConstants.LEFT_FOLLOWER_1_PORT),
            false)
        .addSlaveSpark(FollowerUtils.createFollowerSpark(DriveConstants.LEFT_FOLLOWER_2_PORT),
            false)
        .setEncoderCreator(
            BackupEncoder.creator(
                QuadEncoder.creator(
                    new Encoder(
                        DriveConstants.LEFT_EXTERNAL_FWD_PORT,
                        DriveConstants.LEFT_EXTERNAL_REV_PORT, false),
                    1, 1, 1),
                NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING),
                0.01))
        .build();
  }
}
