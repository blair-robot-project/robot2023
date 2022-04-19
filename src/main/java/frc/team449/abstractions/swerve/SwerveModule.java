package frc.team449.abstractions.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team449.system.motor.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class SwerveModule {
  private final WrappedMotor drivingMotor;
  private final WrappedMotor turningMotor;

  private final PIDController driveController;
  private final ProfiledPIDController turnController;
  private final SimpleMotorFeedforward driveFeedforward;
  private final SimpleMotorFeedforward turnFeedforward;

  /** The location of the module relative to the center */
  public final Translation2d location;

  public SwerveModule(
      @NotNull WrappedMotor drivingMotor,
      @NotNull WrappedMotor turningMotor,
      @NotNull PIDController driveController,
      @NotNull ProfiledPIDController turnController,
      @NotNull SimpleMotorFeedforward driveFeedforward,
      @NotNull SimpleMotorFeedforward turnFeedforward,
      @NotNull Translation2d location) {
    this.drivingMotor = drivingMotor;
    this.turningMotor = turningMotor;
    this.driveController = driveController;
    this.turnController = turnController;
    this.driveFeedforward = driveFeedforward;
    this.turnFeedforward = turnFeedforward;
    this.location = location;

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drivingMotor.getVelocity(), new Rotation2d(turningMotor.getPosition()));
  }

  /** Set the desired state for this module */
  void set(SwerveModuleState desiredState) {
    // Ensure the module doesn't turn the long way around
    var state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getPosition()));

    var drivePid =
        driveController.calculate(drivingMotor.getVelocity(), state.speedMetersPerSecond);
    var driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);
    drivingMotor.setVoltage(drivePid + driveFF);

    var turnPid = turnController.calculate(turningMotor.getVelocity(), state.angle.getRadians());
    var turnFF = turnFeedforward.calculate(turnController.getSetpoint().velocity);
    turningMotor.setVoltage(turnPid + turnFF);
  }
}
