package frc.team1816.robot.commands;

import com.edinarobotics.utils.gamepad.Gamepad;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.Controls;
import frc.team1816.robot.subsystems.Intake;

public class GamepadIntakeCommand extends Command {
    private Intake intake;
    private Gamepad gamepad;
    private double velocity = 0.5;

    public GamepadIntakeCommand(){
        super("gamepadintakecommand");
        this.intake = Components.getInstance().intake;
        this.gamepad = Controls.getInstance().gamepadDriver;
        requires(intake);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if(gamepad.leftBumper().get()) {
            this.intake.setIntake(velocity);
        } else if (gamepad.rightBumper().get()){
            this.intake.setIntake(-velocity);
        } else {
            this.intake.setIntake(0);
        }
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
