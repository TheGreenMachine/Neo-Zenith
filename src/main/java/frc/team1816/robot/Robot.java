package frc.team1816.robot;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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

import java.text.SimpleDateFormat;
import java.util.Date;

public class Robot extends TimedRobot {
    public static final String LOG_DRIVETRAIN_LEFTVEL = "Drivetrain/LeftVel";
    public static final String LOG_DRIVETRAIN_RIGHTVEL = "Drivetrain/RightVel";
    public static final String LOG_ARM_POS = "Arm/Position";
    public static final String LOG_GAMEPAD_LEFTPOWER = "Gamepad/LeftPower";
    public static final String LOG_GAMEPAD_RIGHTPOWER = "Gamepad/RightPower";
    public static final String LOG_GAMEPAD_VELOCITY_MODE = "Gamepad/VelocityMode";

    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Shooter shooter;
    private BadLog log;
    private NetworkTable table;
    private static double startTime;

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

        startTime = System.currentTimeMillis();

        // -- setup the log filename
        SimpleDateFormat fmt = new SimpleDateFormat("yyyyMMdd-HHmm");
        String timestr = fmt.format(new Date());
        String filename = "/home/lvuser/" + timestr + ".bag";
        System.out.println("Writing log data to file: " + filename);

        // -- setup the log file
        log = BadLog.init(filename);
        BadLog.createValue("StartTime", timestr);
        BadLog.createTopic("Time", "sec", () -> getElapsedTime(),
                "xaxis", "hide");
        BadLog.createTopicSubscriber(LOG_DRIVETRAIN_LEFTVEL, "NativeUnits", DataInferMode.DEFAULT);
        BadLog.createTopicSubscriber(LOG_DRIVETRAIN_RIGHTVEL, "NativeUnits", DataInferMode.DEFAULT);

        BadLog.createTopicSubscriber(LOG_ARM_POS, "ohms", DataInferMode.DEFAULT);

        BadLog.createTopicSubscriber(LOG_GAMEPAD_LEFTPOWER, "%", DataInferMode.DEFAULT);
        BadLog.createTopicSubscriber(LOG_GAMEPAD_RIGHTPOWER, "%", DataInferMode.DEFAULT);
        BadLog.createTopicSubscriber(LOG_GAMEPAD_VELOCITY_MODE, "T/F", DataInferMode.DEFAULT);

        log.setDoubleToStringFunction( (d) -> String.format("%.3f", d) );
        log.finishInitialization();
    }

    public static double getElapsedTime() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
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

        // update all log data
        log.updateTopics();
        // only write logs if the DriverStation is enabled
        if (!DriverStation.getInstance().isDisabled()) {
            log.log();
        }
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() { }
}