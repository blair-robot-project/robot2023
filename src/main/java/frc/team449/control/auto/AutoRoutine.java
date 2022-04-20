package frc.team449.control.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutine {
  public final String name;
  public final Trajectory traj;
  public final Command command;

  public AutoRoutine(String name, Trajectory traj, Command command) {
    this.name = name;
    this.traj = traj;
    this.command = command;
  }
}
