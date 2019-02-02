package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team1816.robot.Robot;

public class Arm extends Subsystem {
    private TalonSRX armTalon;

    private double armSpeed;
    private static final int FORWARD_SENSOR_LIMIT = 1475;
    private static final int REVERSE_SENSOR_LIMIT = 915;

    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 30;

    private double kF = 0;
    private double kP = 5.0;
    private double kI = 0;
    private double kD = 1.0;

    private AnalogPotentiometer potentiometer;
    private double armPosition;
    private double armSetpoint;

    public Arm(int armTalon, int potentiometer){
        super();
        this.armTalon = new TalonSRX(armTalon);
        this.armTalon.setNeutralMode(NeutralMode.Brake);
        this.armTalon.setInverted(true);
        this.potentiometer = new AnalogPotentiometer(potentiometer);
        this.armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                kPIDLoopIdx, kTimeoutMs);

        this.armTalon.setSensorPhase(true);

        /* Config the peak and nominal outputs, 12V means full */
        this.armTalon.configNominalOutputForward(0, kTimeoutMs);
        this.armTalon.configNominalOutputReverse(0, kTimeoutMs);
        this.armTalon.configPeakOutputForward(1, kTimeoutMs);
        this.armTalon.configPeakOutputReverse(-1, kTimeoutMs);

        this.armTalon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        this.armTalon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        this.armTalon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        this.armTalon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

        this.armTalon.configAllowableClosedloopError(kPIDLoopIdx, 50, kTimeoutMs);

        this.armTalon.configForwardSoftLimitEnable(true);
        this.armTalon.configReverseSoftLimitEnable(true);
        this.armTalon.configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT, kTimeoutMs);
        this.armTalon.configReverseSoftLimitThreshold(REVERSE_SENSOR_LIMIT, kTimeoutMs);

        int absolutePosition = this.armTalon.getSensorCollection().getPulseWidthPosition();

        /* Mask out overflows, keep bottom 12 bits */
        absolutePosition &= 0xFFF;

        /* Set the quadrature (relative) sensor to match absolute */
        this.armTalon.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

        this.armSetpoint = getArmPosition();
    }

    public void setArm(double armSpeed){
//        if (((getArmPos() < LOWER_THRESHOLD) && (armSpeed < 0)) || ((getArmPos() > UPPER_THRESHOLD) && (armSpeed > 0))){
//            this.armSpeed = 0;
//        } else {
            this.armSpeed = armSpeed;
//        }
        this.armTalon.set(ControlMode.PercentOutput, this.armSpeed);
    }

    public void setArmPosition(double armPosition) {
        this.armSetpoint = armPosition;
        armTalon.set(ControlMode.Position, armSetpoint);
    }

    public double getArmPosition() {
        return armTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getArmSetpoint() {
        return armSetpoint;
    }

    public double getArmPos() {
        return armPosition;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.armTalon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        this.armTalon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        this.armTalon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        this.armTalon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    }

    public double getkF() {
        return kF;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm/kF", this::getkF, this::setkF);
        builder.addDoubleProperty("Arm/kP", this::getkP, null);
        builder.addDoubleProperty("Arm/kI", this::getkI, null);
        builder.addDoubleProperty("Arm/kD", this::getkD, null);
        builder.addDoubleProperty("Arm/ArmPosition/real", this::getArmPosition, null);
    }

    @Override
    public void periodic() {
        armPosition = potentiometer.get();
        System.out.println("Arm.ControlMode = " + armTalon.getControlMode());
        System.out.println("Arm.CurrentPosition = " + armTalon.getSelectedSensorPosition());
        System.out.println("Arm.TargetPosition = " + armTalon.getClosedLoopTarget());
        System.out.println("Arm.ClosedLoopError = " + armTalon.getClosedLoopError(0));
        System.out.println("Arm.MotorOutput = " + armTalon.getMotorOutputPercent());
        BadLog.publish(Robot.LOG_ARM_POS, armPosition);
    }

    @Override
    protected void initDefaultCommand() { }
}
