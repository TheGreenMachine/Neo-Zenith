package frc.team1816.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private NetworkTable table;

    @Override
    public void robotInit() {
        Components.getInstance();
        Controls.getInstance();

        table = NetworkTableInstance.getDefault().getTable("Shuffleboard_PID");
        drivetrain = Components.getInstance().drivetrain;
        arm = Components.getInstance().arm;
        intake = Components.getInstance().intake;
        shooter = Components.getInstance().shooter;

        table.getEntry("kP").setDouble(drivetrain.kP);
        table.getEntry("kI").setDouble(drivetrain.kI);
        table.getEntry("kD").setDouble(drivetrain.kD);
        table.getEntry("kF").setDouble(drivetrain.kF);
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

        
        double pValue = table.getEntry("kP").getDouble(drivetrain.kP);
        double iValue = table.getEntry("kI").getDouble(drivetrain.kI);
        double dValue = table.getEntry("kD").getDouble(drivetrain.kD);
        double fValue = table.getEntry("kF").getDouble(drivetrain.kF);
        drivetrain.setPID(pValue, iValue, dValue, fValue);
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopPeriodic() {
        //System.out.println("Potentiometer: " + arm.getArmPos());
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() { }
}