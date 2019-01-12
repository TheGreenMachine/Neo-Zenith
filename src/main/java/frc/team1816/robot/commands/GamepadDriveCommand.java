package frc.team1816.robot.commands;

import com.edinarobotics.utils.math.Math1816;
import com.edinarobotics.utils.gamepad.Gamepad;
import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.Controls;
import frc.team1816.robot.subsystems.Drivetrain;

public class GamepadDriveCommand extends Command {
    private Drivetrain drivetrain;
    private Gamepad gamepad;

    public GamepadDriveCommand(){
        super("gamepaddrivecommand");
        this.drivetrain = Components.getInstance().drivetrain;
        this.gamepad = Controls.getInstance().gamepadDriver;
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        double speed = Controls.getInstance().getDriveSpeed();
        double turn = Controls.getInstance().getDriveTurn();

        if (gamepad.diamondUp().get()){
            drivetrain.setDrivetrain(0.2, 0.2);
            //System.out.println("Button pressed");
        }

        //System.out.println("Gamepad LeftY " + gamepad.getLeftY() + " RightX " + gamepad.getRightX());

        double leftPower = Math1816.coerceValue(1, -1, speed + turn);
        double rightPower = Math1816.coerceValue(1, -1, speed - turn);
        
        //System.out.println("Left Power: " + leftPower + " Right Power: " + rightPower);

        if (turn == 0){
            drivetrain.setDrivetrain( leftPower,  rightPower);
        } else if (turn !=0 ) {
            drivetrain.setDrivetrain( leftPower,  rightPower);
        }
    }

    @Override
    protected void end() {
        this.drivetrain.setDrivetrain(0,0);
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
