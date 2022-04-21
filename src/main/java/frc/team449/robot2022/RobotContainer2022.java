package frc.team449.robot2022;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team449.control.auto.AutoRoutine;
import frc.team449.control.holonomic.SwerveDrive;
import frc.team449.robot2022.drive.DriveConstants;
import frc.team449.system.AHRS;
import frc.team449.system.encoder.AbsoluteEncoder;
import frc.team449.system.encoder.BackupEncoder;
import frc.team449.system.encoder.NEOEncoder;
import frc.team449.system.encoder.QuadEncoder;
import frc.team449.system.motor.SparkMaxConfig;
import frc.team449.system.motor.WrappedMotor;
import io.github.oblarg.oblog.Loggable;

public final class RobotContainer2022 implements Loggable {

  // Other CAN IDs
  public static final int PDP_CAN = 1, PCM_MODULE = 0;

  public final XboxController driveJoystick = new XboxController(0);

  public final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  public final SwerveDrive drive;
  // Instantiate/declare PDP and other stuff here

  public final Field2d field = new Field2d();
  public final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();

  public RobotContainer2022() {
    SmartDashboard.putData(field);
    this.drive = createDrivetrain();
  }

  /** Helper to make turning motors for swerve */
  private static WrappedMotor makeDrivingMotor(
    String name,
    int motorId,
    boolean inverted,
    Encoder wpiEnc
  ) {
    return new SparkMaxConfig()
      .setName(name + "Drive")
      .setId(motorId)
      .setEnableBrakeMode(true)
      .setInverted(inverted)
      .setEncoderCreator(
        BackupEncoder.creator(
          QuadEncoder.creator(
            wpiEnc,
            DriveConstants.DRIVE_EXT_ENC_CPR,
            DriveConstants.DRIVE_UPR,
            1
          ),
          NEOEncoder.creator(
            DriveConstants.DRIVE_UPR,
            DriveConstants.DRIVE_GEARING
          ),
          DriveConstants.DRIVE_ENC_VEL_THRESHOLD
        )
      )
      .build();
  }

  /** Helper to make turning motors for swerve */
  private static WrappedMotor makeTurningMotor(
    String name,
    int motorId,
    boolean inverted,
    int encoderChannel,
    double offset
  ) {
    return new SparkMaxConfig()
      .setName(name + "Turn")
      .setId(motorId)
      .setEnableBrakeMode(true)
      .setInverted(inverted)
      .setEncoderCreator(
        AbsoluteEncoder.creator(
          encoderChannel,
          2 * Math.PI,
          offset,
          DriveConstants.TURN_UPR,
          DriveConstants.TURN_GEARING
        )
      )
      .build();
  }

  private SwerveDrive createDrivetrain() {
    // todo actually make the modules
    return SwerveDrive.squareDrive(
      ahrs,
      DriveConstants.MAX_LINEAR_SPEED,
      DriveConstants.MAX_ROT_SPEED,
      makeDrivingMotor(
        "FL",
        DriveConstants.FRONT_LEFT_DRIVE,
        false,
        DriveConstants.DRIVE_ENC_FL
      ),
      makeDrivingMotor(
        "FR",
        DriveConstants.FRONT_RIGHT_DRIVE,
        false,
        DriveConstants.DRIVE_ENC_FR
      ),
      makeDrivingMotor(
        "BL",
        DriveConstants.BACK_LEFT_DRIVE,
        false,
        DriveConstants.DRIVE_ENC_BL
      ),
      makeDrivingMotor(
        "BR",
        DriveConstants.BACK_RIGHT_DRIVE,
        false,
        DriveConstants.DRIVE_ENC_BR
      ),
      makeTurningMotor(
        "FL",
        DriveConstants.FRONT_LEFT_TURN,
        false,
        DriveConstants.TURN_ENC_CHAN_FL,
        DriveConstants.TURN_ENC_OFFSET_FL
      ),
      makeTurningMotor(
        "FR",
        DriveConstants.FRONT_LEFT_TURN,
        false,
        DriveConstants.TURN_ENC_CHAN_FR,
        DriveConstants.TURN_ENC_OFFSET_FR
      ),
      makeTurningMotor(
        "BL",
        DriveConstants.FRONT_LEFT_TURN,
        false,
        DriveConstants.TURN_ENC_CHAN_BL,
        DriveConstants.TURN_ENC_OFFSET_BL
      ),
      makeTurningMotor(
        "BR",
        DriveConstants.FRONT_LEFT_TURN,
        false,
        DriveConstants.TURN_ENC_CHAN_BR,
        DriveConstants.TURN_ENC_OFFSET_BR
      ),
      DriveConstants.FRONT_LEFT_LOC,
      () -> new PIDController(0, 0, 0),
      () ->
        new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(0, 0)
        ),
      new SimpleMotorFeedforward(0, 0, 0),
      new SimpleMotorFeedforward(0, 0, 0)
    );
  }

  public void teleopInit() {
    // todo Add button bindings here
  }

  public void robotPeriodic() {
    // todo Update robot pose on field and stuff
  }

  public void simulationPeriodic() {
    // Update simulated mechanisms on Mechanism2d widget and stuff
  }
}
