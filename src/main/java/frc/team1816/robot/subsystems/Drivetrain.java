package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team1816.robot.Robot;

public class Drivetrain extends Subsystem {
    private PigeonIMU gyro;

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
    public double kP = 5.0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0.128;
    public double leftTalonVelocity, rightTalonVelocity, leftTalonPosition, rightTalonPosition;

    private boolean isPercentOutput;

    public Drivetrain(int leftMain, int leftSlaveOne, int leftSlaveTwo, int rightMain, int rightSlaveOne,
                      int rightSlaveTwo) {
        super();
        this.gyro = new PigeonIMU(this.leftSlaveTwo);

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

        BadLog.createTopic(Robot.LOG_DRIVETRAIN_LEFTVEL, "NativeUnits",
                () -> getLeftVelocity(), "hide", "join:Drivetrain/Velocities");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_RIGHTVEL, "NativeUnits",
                () -> getRightVelocity(), "hide", "join:Drivetrain/Velocities");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_P, BadLog.UNITLESS,
                () -> getP(), "join:Drivetrain/PID", "hide");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_I, BadLog.UNITLESS,
                this::getI,"join:Drivetrain/PID", "hide");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_D, BadLog.UNITLESS,
                this::getD,"join:Drivetrain/PID", "hide");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_F, BadLog.UNITLESS,
                this::getF,"join:Drivetrain/PID", "hide");

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
        return leftTalonVelocity;
    }

    public double getRightVelocity(){
        return rightTalonVelocity;
    }

    public double getLeftPosition(){
        return leftTalonPosition;
    }

    public double getRightPosition(){
        return rightTalonPosition;
    }

    @Override
    public void periodic() {
        leftTalonVelocity = leftMain.getSelectedSensorVelocity(0);
        rightTalonVelocity = rightMain.getSelectedSensorVelocity(0);
        leftTalonPosition = leftMain.getSelectedSensorPosition(0);
        rightTalonPosition = rightMain.getSelectedSensorPosition(0);

        if (isPercentOutput){
            this.leftMain.set(ControlMode.PercentOutput, leftPower);
            this.rightMain.set(ControlMode.PercentOutput, rightPower);
        } else {
            this.leftMain.set(ControlMode.Velocity, MAX_VELOCITY_TICKS_PER_100MS * leftPower);
            this.rightMain.set(ControlMode.Velocity, MAX_VELOCITY_TICKS_PER_100MS * rightPower);
        }
        System.out.println("Left Velocity: " + getLeftVelocity() +
                            " Right Velocity: " + getRightVelocity());
    }

    public void setPID(double pValue, double iValue, double dValue, double fValue){
        this.kP = pValue;
        this.kI = iValue;
        this.kD = dValue;
        this.kF = fValue;
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
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("LeftTalonVelocity", this::getLeftVelocity, null);  
        builder.addDoubleProperty("RightTalonVelocity", this::getRightVelocity, null);        
        builder.addDoubleProperty("LeftTalonPosition", this::getLeftPosition, null);
        builder.addDoubleProperty("RightTalonPosition", this::getRightPosition, null);
    }
      
    public double getGyroAngle() {
        return gyro.getFusedHeading();
    }

    @Override
    protected void initDefaultCommand() { }

    double getP() { return kP; }
    double getI() { return kI; }
    double getD() { return kD; }
    double getF() { return kF; }

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
