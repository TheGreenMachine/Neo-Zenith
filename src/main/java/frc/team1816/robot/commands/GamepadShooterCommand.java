package frc.team1816.robot.commands;

import com.edinarobotics.utils.gamepad.Gamepad;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.subsystems.Shooter;

public class GamepadShooterCommand extends Command {
    private Shooter shooter;

    public GamepadShooterCommand(){
        super("gamepadshootercommand");
        this.shooter = Components.getInstance().shooter;
        requires(shooter);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        shooter.shoot();
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
