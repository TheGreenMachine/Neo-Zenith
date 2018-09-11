package frc.team1816.robot.commands;

import com.edinarobotics.utils.gamepad.Gamepad;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.Controls;
import frc.team1816.robot.subsystems.Arm;

public class GamepadArmCommand extends Command {
    private Arm arm;
    private double power = 0.5;
    private Gamepad gamepad;

    public GamepadArmCommand(){
        super("gamepadarmcommand");
        this.arm = Components.getInstance().arm;
        this.gamepad = Controls.getInstance().gamepadDriver;

        requires(arm);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if(gamepad.dPadUp().get()) {
            this.arm.setArm(power);
        } else if (gamepad.dPadDown().get()){
            this.arm.setArm(-power);
        } else {
            this.arm.setArm(0);
        }
    }

    @Override
    protected void end() {
        arm.setArm(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
