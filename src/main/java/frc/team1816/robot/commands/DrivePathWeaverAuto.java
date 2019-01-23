package frc.team1816.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DrivePathWeaverAuto extends CommandGroup {
 
  public DrivePathWeaverAuto() {
      addSequential(new DrivePathWeaverCommand());
  }
}
