package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team1816.robot.Robot;

import java.beans.Introspector;
import java.beans.PropertyDescriptor;

public class Drivetrain extends Subsystem {
    private TalonSRX leftMain, leftSlaveOne, leftSlaveTwo, rightMain, rightSlaveOne, rightSlaveTwo;
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final double kDriveVoltageRampRate = 0.0;

    public static double TICKS_PER_REV;
    public static double TICKS_PER_INCH;

    public static double DRIVETRAIN_WIDTH;
    public static double INCHES_PER_REV;
    public static double MAX_VELOCITY_TICKS_PER_100MS = 8000;

    private double leftPower, rightPower = 0;
    public double kP = 0; 
    public double kI = 0;
    public double kD = 0;
    public double kF = 0.128;
    public double leftVel, rightVel;

    private boolean isPercentOutput;

    public Drivetrain(int leftMain, int leftSlaveOne, int leftSlaveTwo, int rightMain, int rightSlaveOne,
                      int rightSlaveTwo) {
        super();
        this.leftMain =  TalonSRXFactory.createDefaultTalon(leftMain);
        this.leftSlaveOne = TalonSRXFactory.createPermanentSlaveTalon(leftSlaveOne, leftMain);
        this.leftSlaveTwo = TalonSRXFactory.createPermanentSlaveTalon(leftSlaveTwo, leftMain);
        configureMaster(this.leftMain, true);
        this.leftSlaveOne.setInverted(true);
        this.leftSlaveTwo.setInverted(true);

        this.rightMain = TalonSRXFactory.createDefaultTalon(rightMain);
        this.rightSlaveOne = TalonSRXFactory.createPermanentSlaveTalon(rightSlaveOne, rightMain);
        this.rightSlaveTwo = TalonSRXFactory.createPermanentSlaveTalon(rightSlaveTwo, rightMain);
        configureMaster(this.rightMain, false);

        this.leftMain.setNeutralMode(NeutralMode.Brake);
        this.leftSlaveOne.setNeutralMode(NeutralMode.Brake);
        this.leftSlaveTwo.setNeutralMode(NeutralMode.Brake);

        this.rightMain.setNeutralMode(NeutralMode.Brake);
        this.rightSlaveOne.setNeutralMode(NeutralMode.Brake);
        this.rightSlaveTwo.setNeutralMode(NeutralMode.Brake);

        this.setPID(kP, kI, kD, kF);

        this.leftMain.selectProfileSlot(0,0);
        this.rightMain.selectProfileSlot(0,0);
    }

    public void setDrivetrainVelocity(double leftPower, double rightPower){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        isPercentOutput = false;
    }

    public void setDrivetrainPercentOutput(double leftPower, double rightPower){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        isPercentOutput = true;
    }

    public double getLeftVelocity(){
        return leftVel;
    }

    public double getRightVelocity(){
        return rightVel;
    }

    @Override
    public void periodic() {
        leftVel = leftMain.getSelectedSensorVelocity(0);
        rightVel = rightMain.getSelectedSensorVelocity(0);

        if (isPercentOutput){
            this.leftMain.set(ControlMode.PercentOutput, leftPower);
            this.rightMain.set(ControlMode.PercentOutput, rightPower);
        } else {
            this.leftMain.set(ControlMode.Velocity, MAX_VELOCITY_TICKS_PER_100MS * leftPower);
            this.rightMain.set(ControlMode.Velocity, MAX_VELOCITY_TICKS_PER_100MS * rightPower);
        }
        System.out.println("Left Velocity: " + getLeftVelocity() +
                            " Right Velocity: " + getRightVelocity());
        BadLog.publish(Robot.LOG_DRIVETRAIN_LEFTVEL, leftVel);
        BadLog.publish(Robot.LOG_DRIVETRAIN_RIGHTVEL, rightVel);
    }

    public void setPID(double pValue, double iValue, double dValue, double fValue){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        System.out.printf("PID values set: P=%.2f, I=%.2f, D=%.2f, F=%.2f\n", kP, kI, kD, kF);

        this.leftMain.config_kP(0, kP, 20);
        this.leftMain.config_kI(0, kI, 20);
        this.leftMain.config_kD(0, kD, 20);
        this.leftMain.config_kF(0, kF, 20);
        //this.leftMain.config_IntegralZone(0, izone, 20);

        this.rightMain.config_kP(0, kP, 20);
        this.rightMain.config_kI(0, kI, 20);
        this.rightMain.config_kD(0, kD, 20);
        this.rightMain.config_kF(0, kF, 20);
        //this.rightMain.config_IntegralZone(0, izone, 20);

        BadLog.publish(Robot.LOG_DRIVETRAIN_PID_P, kP);
        BadLog.publish(Robot.LOG_DRIVETRAIN_PID_I, kI);
        BadLog.publish(Robot.LOG_DRIVETRAIN_PID_D, kD);
        BadLog.publish(Robot.LOG_DRIVETRAIN_PID_F, kF);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("LeftVelocity", this::getLeftVelocity, null);  
        builder.addDoubleProperty("RightVelocity", this::getRightVelocity, null);        
      
    }

    @Override
    protected void initDefaultCommand() { }

    private void configureMaster(TalonSRX talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            System.out.println("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent);
        }
        talon.setInverted(left);
        talon.setSensorPhase(false);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, kLongCANTimeoutMs);
        talon.configClosedloopRamp(kDriveVoltageRampRate, kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }
}
