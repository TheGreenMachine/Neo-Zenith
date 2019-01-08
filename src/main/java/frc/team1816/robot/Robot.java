package frc.team1816.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team1816.robot.commands.GamepadArmCommand;
import frc.team1816.robot.commands.GamepadDriveCommand;
import frc.team1816.robot.commands.GamepadIntakeCommand;
import frc.team1816.robot.commands.GamepadShooterCommand;
import frc.team1816.robot.subsystems.Arm;
import frc.team1816.robot.subsystems.Drivetrain;
import frc.team1816.robot.subsystems.Intake;
import frc.team1816.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Shooter shooter;


    @Override
    public void robotInit() {
        Components.getInstance();
        Controls.getInstance();

        drivetrain = Components.getInstance().drivetrain;
        arm = Components.getInstance().arm;
        intake = Components.getInstance().intake;
        shooter = Components.getInstance().shooter;

    }

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void teleopInit() {
        drivetrain.setDefaultCommand(new GamepadDriveCommand());
        arm.setDefaultCommand(new GamepadArmCommand());
        intake.setDefaultCommand(new GamepadIntakeCommand());
        shooter.setDefaultCommand(new GamepadShooterCommand());
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
        System.out.println("Potentiometer: " + arm.getArmPos());
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() { }
}