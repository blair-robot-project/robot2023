package frc.team449.abstractions.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team449.abstractions.DriveSubsystem;
import frc.team449.system.AHRS;

public class TankDrive implements DriveSubsystem {

  @Override
  public void set(ChassisSpeeds desiredSpeeds) {
    // TODO Auto-generated method stub

  }

  @Override
  public AHRS getAHRS() {
    // todo implement
    return null;
  }

  @Override
  public void setPose(Pose2d pose) {
    // TODO Auto-generated method stub
  }

  @Override
  public Pose2d getPose() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }
}
