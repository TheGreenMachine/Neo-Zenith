package frc.team1816.robot.commands;

import com.edinarobotics.utils.gamepad.Gamepad;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Arm;

public class GamepadArmCommand extends Command {
    private Arm arm;
    private double power = Components.getInstance().ARM_SPEED_DEMO;
    private Gamepad gamepad;

    public GamepadArmCommand(Gamepad gamepad){
        super("raisearmcommand");
        this.arm = Components.getInstance().arm;
        this.gamepad = gamepad;

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
