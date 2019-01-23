package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team1816.robot.Robot;

public class Drivetrain extends Subsystem {
    private PigeonIMU gyro;

    private TalonSRX leftMain, leftSlaveOne, leftSlaveTwo, rightMain, rightSlaveOne, rightSlaveTwo;
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final double kDriveVoltageRampRate = 0.0;

    public static double TICKS_PER_INCH = 439;
    public static double TICKS_PER_REV = 9600;

    public static double DRIVETRAIN_WIDTH = 21.75; //Need to double check
    public static double MAX_VELOCITY_TICKS_PER_100MS = 8000;

    private double leftPower, rightPower = 0;
    public double kP = 0.1;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0.128;
    private double leftTalonVelocity, rightTalonVelocity, leftTalonPosition, rightTalonPosition, gyroAngle;
    private double leftTalonError, rightTalonError;

    private double xPos, yPos, prevX, prevY, prevLeftInches, prevRightInches, initAngle;

    private NetworkTable positions;

    private boolean isPercentOutput;

    public Drivetrain(int leftMainID, int leftSlaveOneID, int leftSlaveTwoID, int rightMainID, int rightSlaveOneID,
                      int rightSlaveTwoID) {
        super();

        this.positions = NetworkTableInstance.getDefault().getTable("positions");
        this.leftMain =  TalonSRXFactory.createDefaultTalon(leftMainID);
        this.leftSlaveOne = TalonSRXFactory.createPermanentSlaveTalon(leftSlaveOneID, leftMainID);
        this.leftSlaveTwo = TalonSRXFactory.createPermanentSlaveTalon(leftSlaveTwoID, leftMainID);
        configureMaster(this.leftMain, true);
        this.leftSlaveOne.setInverted(true);
        this.leftSlaveTwo.setInverted(true);

        this.rightMain = TalonSRXFactory.createDefaultTalon(rightMainID);
        this.rightSlaveOne = TalonSRXFactory.createPermanentSlaveTalon(rightSlaveOneID, rightMainID);
        this.rightSlaveTwo = TalonSRXFactory.createPermanentSlaveTalon(rightSlaveTwoID, rightMainID);
        configureMaster(this.rightMain, false);

        this.leftMain.setNeutralMode(NeutralMode.Brake);
        this.leftSlaveOne.setNeutralMode(NeutralMode.Brake);
        this.leftSlaveTwo.setNeutralMode(NeutralMode.Brake);

        this.rightMain.setNeutralMode(NeutralMode.Brake);
        this.rightSlaveOne.setNeutralMode(NeutralMode.Brake);
        this.rightSlaveTwo.setNeutralMode(NeutralMode.Brake);

        this.gyro = new PigeonIMU(this.leftSlaveTwo);

        initCoordinateTracking();

        this.setPID(kP, kI, kD, kF);

        this.leftMain.selectProfileSlot(0,0);
        this.rightMain.selectProfileSlot(0,0);

        BadLog.createTopic(Robot.LOG_DRIVETRAIN_LEFT_VEL, "NativeUnits",
                this::getLeftVelocity, "hide", "join:Drivetrain/Velocities");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_RIGHT_VEL, "NativeUnits",
                this::getRightVelocity, "hide", "join:Drivetrain/Velocities");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_LEFT_VEL_INPUT, "NativeUnits",
                this::getLeftPower, "hide", "join:Drivetrain/VelocityInputs");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_RIGHT_VEL_INPUT, "NativeUnits",
                this::getRightPower, "hide", "join:Drivetrain/VelocityInputs");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_LEFT_POSITION, "NativeUnits",
                this::getLeftPosition, "hide", "join:Drivetrain/TalonPosition");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_RIGHT_POSITION, "NativeUnits",
                this::getRightPosition, "hide", "join:Drivetrain/TalonPosition");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_LEFT_ERROR, "NativeUnits",
                this::getLeftError, "hide", "join:Drivetrain/TalonError");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_RIGHT_ERROR, "NativeUnits",
                this::getRightError, "hide", "join:Drivetrain/TalonError");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_P, BadLog.UNITLESS,
                this::getP, "join:Drivetrain/PID", "hide");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_I, BadLog.UNITLESS,
                this::getI,"join:Drivetrain/PID", "hide");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_D, BadLog.UNITLESS,
                this::getD,"join:Drivetrain/PID", "hide");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_PID_F, BadLog.UNITLESS,
                this::getF,"join:Drivetrain/PID", "hide");                
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_POSTRACK_X, "inches",
                this::getXPos, "hide", "join:Drivetrain/Coordinates");
        BadLog.createTopic(Robot.LOG_DRIVETRAIN_POSTRACK_Y, "inches",
                this::getYPos, "hide", "join:Drivetrain/Coordinates");
    }

    public void setDrivetrainVelocity(double leftPower, double rightPower){
        this.leftPower = MAX_VELOCITY_TICKS_PER_100MS * leftPower;
        this.rightPower = MAX_VELOCITY_TICKS_PER_100MS * rightPower;
        isPercentOutput = false;
    }

    public void setDrivetrainPercentOutput(double leftPower, double rightPower){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        isPercentOutput = true;
    }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }

    public double getLeftPower(){
        return leftPower;
    }

    public double getRightPower(){
        return rightPower;
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

    public double getLeftInches() {
        return getLeftPosition() / TICKS_PER_INCH;
    }

    public double getRightInches() {
        return getRightPosition() / TICKS_PER_INCH;
    }

    public double getLeftError(){
        return leftTalonError;
    }

    public double getRightError(){
        return rightTalonError;
    }

    public double getGyroAngle() {
        return gyro.getFusedHeading();
    }

    public double getXPos() {
        return  xPos;
    }

    public double getYPos() {
        return yPos;
    }

    public void initCoordinateTracking() {
        xPos = 0;
        yPos = 0;
        prevRightInches = 0.0;
        prevLeftInches = 0.0;
        prevX = 0;
        prevY = 0;
        initAngle = getGyroAngle();
        System.out.println("Initial Angle: " + initAngle);
    }

    @Override
    public void periodic() {
        gyroAngle = getGyroAngle();
        leftTalonVelocity = leftMain.getSelectedSensorVelocity(0);
        rightTalonVelocity = rightMain.getSelectedSensorVelocity(0);
        leftTalonPosition = leftMain.getSelectedSensorPosition(0);
        rightTalonPosition = rightMain.getSelectedSensorPosition(0);
        leftTalonError = leftMain.getClosedLoopError(0);
        rightTalonError = rightMain.getClosedLoopError(0);

        if (isPercentOutput){
            this.leftMain.set(ControlMode.PercentOutput, leftPower);
            this.rightMain.set(ControlMode.PercentOutput, rightPower);
        } else {
            this.leftMain.set(ControlMode.Velocity, leftPower);
            this.rightMain.set(ControlMode.Velocity, rightPower);
        }
        System.out.println("Left Velocity: " + getLeftVelocity() +
                            " Right Velocity: " + getRightVelocity());

        double currLeftInches = getLeftInches();
        double currRightInches = getRightInches();
        double avgDistance = ((currLeftInches - prevLeftInches) + (currRightInches - prevRightInches)) / 2;
        double theta = (Math.toRadians(initAngle - gyroAngle));

        xPos = avgDistance * Math.cos(theta) + prevX;
        yPos = avgDistance * Math.sin(theta) + prevY;

        positions.getEntry("xPos").setDouble(xPos);
        positions.getEntry("yPos").setDouble(yPos);

        prevX = xPos;
        prevY = yPos;
        prevLeftInches = currLeftInches;
        prevRightInches = currRightInches;
    }
    
    public void resetEncoders() {
        this.leftMain.getSensorCollection().setQuadraturePosition(0, 10);
        this.rightMain.getSensorCollection().setQuadraturePosition(0, 10);
    }

    public void resetGyro(){
        this.gyro.setYaw(0, 10);
        this.gyro.setFusedHeading(0, 10);
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
        builder.addDoubleProperty("LeftInches", this::getLeftInches, null);
        builder.addDoubleProperty("RightInches", this::getRightInches, null);
        builder.addDoubleProperty("GyroAngle", this::getGyroAngle, null);
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
