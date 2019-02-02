package frc.team1816.robot;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
    public static final String LOG_DRIVETRAIN_LEFT_VEL = "Drivetrain/LeftVel";
    public static final String LOG_DRIVETRAIN_RIGHT_VEL = "Drivetrain/RightVel";
    public static final String LOG_DRIVETRAIN_LEFT_VEL_INPUT = "Divetrain/LeftVelInput";
    public static final String LOG_DRIVETRAIN_RIGHT_VEL_INPUT = "Drivetrain/RightVelInput";
    public static final String LOG_DRIVETRAIN_LEFT_POSITION = "Drivetrain/LeftPosition";
    public static final String LOG_DRIVETRAIN_RIGHT_POSITION = "Drivetrain/RightPosition";
    public static final String LOG_DRIVETRAIN_LEFT_ERROR = "Drivetrain/LeftError";
    public static final String LOG_DRIVETRAIN_RIGHT_ERROR = "Drivetrain/RightError";
    public static final String LOG_ARM_POS = "Arm/Position";
    public static final String LOG_ARM_READING = "Arm/EncoderPosition";
    public static final String LOG_GAMEPAD_LEFT_POWER = "Gamepad/LeftPower";
    public static final String LOG_GAMEPAD_RIGHT_POWER = "Gamepad/RightPower";
    public static final String LOG_GAMEPAD_VELOCITY_MODE = "Gamepad/VelocityMode";
    public static final String LOG_DRIVETRAIN_PID_P = "Drivetrain/PID_P";
    public static final String LOG_DRIVETRAIN_PID_I = "Drivetrain/PID_I";
    public static final String LOG_DRIVETRAIN_PID_D = "Drivetrain/PID_D";
    public static final String LOG_DRIVETRAIN_PID_F = "Drivetrain/PID_F";
    public static final String LOG_DRIVETRAIN_POSTRACK_X = "Drivetrain/PositionTracking_X";
    public static final String LOG_DRIVETRAIN_POSTRACK_Y = "Drivetrain/PositionTracking_Y";

    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Shooter shooter;

    private DrivePathWeaverAuto drivePathWeaverAuto;
    
    private BadLog log;
    private NetworkTable table;
    private static double startTime;

    @Override
    public void robotInit() {
        // -- setup the log filename
        SimpleDateFormat fmt = new SimpleDateFormat("yyyyMMdd-HHmm");
        String timestr = fmt.format(new Date());
        String filename = "/home/lvuser/" + timestr + ".bag";
        System.out.println("Writing log data to file: " + filename);

        // -- setup the log file
        log = BadLog.init(filename);
        BadLog.createValue("StartTime", timestr);
        BadLog.createTopic("Time", "sec", Robot::getElapsedTime,
                "xaxis", "hide");

        BadLog.createTopicSubscriber(LOG_ARM_POS, "ohms", DataInferMode.DEFAULT);

        BadLog.createTopicSubscriber(LOG_GAMEPAD_LEFT_POWER, "%", DataInferMode.DEFAULT);
        BadLog.createTopicSubscriber(LOG_GAMEPAD_RIGHT_POWER, "%", DataInferMode.DEFAULT);
        BadLog.createTopicSubscriber(LOG_GAMEPAD_VELOCITY_MODE, "T/F", DataInferMode.DEFAULT);

        log.setDoubleToStringFunction((d) -> String.format("%.3f", d));

        Components.getInstance();
        Controls.getInstance();

        table = NetworkTableInstance.getDefault().getTable("Shuffleboard_PID");
        
        drivetrain = Components.getInstance().drivetrain;
        arm = Components.getInstance().arm;
        intake = Components.getInstance().intake;
        shooter = Components.getInstance().shooter;

        drivePathWeaverAuto = new DrivePathWeaverAuto();

        table.getEntry("kP").setDouble(drivetrain.kP);
        table.getEntry("kI").setDouble(drivetrain.kI);
        table.getEntry("kD").setDouble(drivetrain.kD);
        table.getEntry("kF").setDouble(drivetrain.kF);

        table.getEntry("arm_kP").setDouble(arm.getkP());
        table.getEntry("arm_kI").setDouble(arm.getkI());
        table.getEntry("arm_kD").setDouble(arm.getkD());
        table.getEntry("arm_kF").setDouble(arm.getkF());

        startTime = System.currentTimeMillis();
        log.finishInitialization();

    }

    public static double getElapsedTime() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    @Override
    public void disabledInit() { }

    @Override
    public void autonomousInit() {
        drivePathWeaverAuto.start();
     }

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

        double armPValue = table.getEntry("arm_kP").getDouble(arm.getkP());
        double armIValue = table.getEntry("arm_kI").getDouble(arm.getkI());
        double armDValue = table.getEntry("arm_kD").getDouble(arm.getkD());
        arm.setPID(armPValue, armIValue, armDValue);
    }

    @Override
    public void testInit() { }


    @Override
    public void disabledPeriodic() { }
    
    @Override
    public void autonomousPeriodic() {
        // update all log data
        log.updateTopics();
        // only write logs if the DriverStation is enabled
        if (!DriverStation.getInstance().isDisabled()) {
            log.log();
        
        }
        Scheduler.getInstance().run();
     }

    @Override
    public void teleopPeriodic() {
        // update all log data
        log.updateTopics();
        // only write logs if the DriverStation is enabled
        if (!DriverStation.getInstance().isDisabled()) {
            log.log();
        }

//        arm.setArmPosition(table.getEntry("ArmSetPoint").getDouble(arm.getArmSetpoint()));

        System.out.println("Gyro Angle: " + drivetrain.getGyroAngle());
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() { }
}
