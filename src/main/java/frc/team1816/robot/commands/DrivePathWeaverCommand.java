package frc.team1816.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Drivetrain;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class DrivePathWeaverCommand extends Command {
  private Drivetrain drivetrain;

  private String pathName = "TestPath"; 

  private EncoderFollower leftFollower, rightFollower;
  private Notifier followerNotifier;

  public DrivePathWeaverCommand() {
    super("drivepathweavercommand");
    this.drivetrain = Components.getInstance().drivetrain;
    requires(drivetrain);
  }

  @Override
  protected void initialize() {
    Trajectory leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
    Trajectory rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");

    leftFollower = new EncoderFollower(leftTrajectory);
    rightFollower = new EncoderFollower(rightTrajectory);

    //leftFollower.configureEncoder(initial_position, ticks_per_revolution, wheel_diameter); //Need to find TICKS_PER_REV
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
