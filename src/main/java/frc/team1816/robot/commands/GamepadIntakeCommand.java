package frc.team1816.robot.commands;

import com.edinarobotics.utils.gamepad.Gamepad;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Intake;

public class GamepadIntakeCommand extends Command {
    private Intake intake;
    private Gamepad gamepad;
    private double velocity = 1;

    public GamepadIntakeCommand(Gamepad gamepad){
        super("gamepadintakecommand");
        this.intake = Components.getInstance().intake;
        this.gamepad = gamepad;
        this.velocity = velocity;
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
