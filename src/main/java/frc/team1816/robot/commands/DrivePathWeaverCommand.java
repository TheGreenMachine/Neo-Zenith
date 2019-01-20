package frc.team1816.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class DrivePathWeaverCommand extends Command {
  private Drivetrain drivetrain;

  private String pathName = "TestPath"; 

  private EncoderFollower leftFollower, rightFollower;
  //private Notifier followerNotifier; //Don't think it is required

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

    double maxVelocityInches = 10 * drivetrain.MAX_VELOCITY_TICKS_PER_100MS / drivetrain.TICKS_PER_INCH; //Converts units to in/s

    leftFollower.configureEncoder((int)drivetrain.getLeftPosition(), (int)drivetrain.TICKS_PER_REV, drivetrain.DRIVETRAIN_WIDTH); //Need to find TICKS_PER_REV
    leftFollower.configurePIDVA(drivetrain.kP, drivetrain.kI, drivetrain.kD, 1/maxVelocityInches, 0); //Need to tune PID

    rightFollower.configureEncoder((int)drivetrain.getRightPosition(), (int)drivetrain.TICKS_PER_REV, drivetrain.DRIVETRAIN_WIDTH); //Need to find TICKS_PER_REV
    rightFollower.configurePIDVA(drivetrain.kP, drivetrain.kI, drivetrain.kD, 1/maxVelocityInches, 0); // Need to tune PID
  }

  @Override
  protected void execute() {
    double leftSpeed = leftFollower.calculate((int)drivetrain.getLeftPosition());
    double rightSpeed = rightFollower.calculate((int)drivetrain.getRightPosition());
    
    double currentHeading = drivetrain.getGyroAngle();
    double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
    double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - currentHeading);
    double turn = 0.8 * (-1.0/80.0) * headingDifference; //Error adjustment: Formula given in Jaci's documentation

    drivetrain.setDrivetrainPercentOutput(leftSpeed + turn, rightSpeed - turn);
  }

  @Override
  protected boolean isFinished() {
    if ((leftFollower.isFinished()) || (rightFollower.isFinished())){
      return true;
    } else {
      return false;
    }
  }

  @Override
  protected void end() {
    drivetrain.setDrivetrainPercentOutput(0, 0);
  }

  @Override
  protected void interrupted() {
  }
}
