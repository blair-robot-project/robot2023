package frc.team449.control.differential;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveCommand extends CommandBase {

  private final DifferentialDrive drivetrain;
  private final Supplier<ChassisSpeeds> controllerInput;

  public DriveCommand(DifferentialDrive drivetrain, Supplier<ChassisSpeeds> controllerInput){
    this.drivetrain = drivetrain;
    this.controllerInput = controllerInput;
    addRequirements(drivetrain);
  }
  @Override
  public void execute() {
    drivetrain.set(controllerInput.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
