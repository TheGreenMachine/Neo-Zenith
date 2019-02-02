package frc.team1816.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Arm;

public class GamepadArmPositionCommand extends Command {
    private int setpoint;
    private Arm arm;

    public GamepadArmPositionCommand(int setpoint) {
        super("gamepadarmpositioncommand");
        this.setpoint = setpoint;
        this.arm = Components.getInstance().arm;
        requires(arm);
    }

    @Override
    protected void execute() {
        arm.setArmPosition(setpoint);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
