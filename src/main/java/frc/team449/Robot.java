package frc.team449;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team449.robot2022.RobotContainer2022;

/**
 * The main class of the robot, constructs all the subsystems and initializes
 * default commands.
 */
public class Robot extends TimedRobot {
  private final RobotContainer2022 robotContainer = new RobotContainer2022();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    robotContainer.robotPeriodic();
  }

  @Override
  public void autonomousInit() {
    var routine = robotContainer.autoChooser.getSelected();
    robotContainer.field
        .getObject(routine.getName())
        .setTrajectory(routine.getTrajectory());
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }
}
