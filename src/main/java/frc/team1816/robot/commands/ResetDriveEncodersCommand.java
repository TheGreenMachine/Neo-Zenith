package frc.team1816.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Drivetrain;

public class ResetDriveEncodersCommand extends Command {
  private Drivetrain drivetrain;

  public ResetDriveEncodersCommand() {
    super("resetdriveencoderscommand");
    this.drivetrain = Components.getInstance().drivetrain;
    requires(drivetrain);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    drivetrain.resetEncoders();
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
