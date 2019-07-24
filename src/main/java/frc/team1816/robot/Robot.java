package frc.team1816.robot;


import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team1816.robot.commands.*;
import frc.team1816.robot.subsystems.Arm;
import frc.team1816.robot.subsystems.Drivetrain;
import frc.team1816.robot.subsystems.Intake;
import frc.team1816.robot.subsystems.Shooter;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Robot extends TimedRobot {

    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Shooter shooter;

    private NetworkTableInstance inst;
    private NetworkTable table;

    public static RobotState stateInstance = new RobotState();

    public static class RobotState {
        public double width = 640;
        public double height = 480;
        public double xCoord = -1.0;

        public double getVisionXCoord() {
            return xCoord;
        }

        public double getVisionWidth() {
            return width;
        }

        public double getVisionHeight() {
            return height;
        }
    }

    @Override
    public void robotInit() {
        Components.getInstance();
        Controls.getInstance();

        drivetrain = Components.getInstance().drivetrain;
        arm = Components.getInstance().arm;
        intake = Components.getInstance().intake;
        shooter = Components.getInstance().shooter;

        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("SmartDashboard");
        NetworkTableEntry widthEntry = table.getEntry("width");
        NetworkTableEntry heightEntry = table.getEntry("height");
        NetworkTableEntry xCoordEntry = table.getEntry("center_x");

        stateInstance.width = widthEntry.getDouble(640.0);
        stateInstance.height = heightEntry.getDouble(480.0);
        stateInstance.xCoord = xCoordEntry.getDouble(-1.0);

        table.addEntryListener("center_x", (table, key, entry, value, flags) -> {stateInstance.xCoord = value.getDouble();}, 
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

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
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() { }
}
