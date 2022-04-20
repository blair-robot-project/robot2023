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
import frc.team449.abstractions.auto.AutoRoutine;
import frc.team449.abstractions.swerve.SwerveDrive;
import frc.team449.robot2022.drive.DriveConstants;
import frc.team449.system.AHRS;
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

  private static WrappedMotor makeDrivingMotor(
      String name, int port, boolean inverted, Encoder wpiEnc) {
    return new SparkMaxConfig()
        .setName(name + "Drive")
        .setPort(port)
        .setEnableBrakeMode(true)
        .setInverted(inverted)
        .setEncoderCreator(
            BackupEncoder.creator(
                QuadEncoder.creator(
                    wpiEnc, DriveConstants.DRIVE_EXT_ENC_CPR, DriveConstants.DRIVE_UPR, 1),
                NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING),
                DriveConstants.DRIVE_ENC_VEL_THRESHOLD))
        .build();
  }

  private SwerveDrive createDrivetrain() {
    // todo actually make the modules
    return SwerveDrive.squareDrive(
        ahrs,
        DriveConstants.MAX_LINEAR_SPEED,
        DriveConstants.MAX_ROT_SPEED,
        makeDrivingMotor(
            "FL", DriveConstants.FRONT_LEFT_DRIVE, false, DriveConstants.FRONT_LEFT_DRIVE_ENC),
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        DriveConstants.FRONT_LEFT_LOC,
        () -> new PIDController(0, 0, 0),
        () -> new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)),
        new SimpleMotorFeedforward(0, 0, 0),
        new SimpleMotorFeedforward(0, 0, 0));
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
