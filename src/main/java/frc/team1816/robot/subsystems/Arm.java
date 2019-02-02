package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.team1816.robot.Robot;

public class Arm extends Subsystem {
    private TalonSRX armTalon;

    public static final int FORWARD_SENSOR_LIMIT = 1475;
    public static final int REVERSE_SENSOR_LIMIT = 915;
    private static final int ALLOWABLE_CLOSED_LOOP_ERROR = 50;

    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 30;

    private double kF = 0;
    private double kP = 5.0;
    private double kI = 0;
    private double kD = 1.0;

    private double armPosition;
    private double armSpeed;

    private boolean outputsChanged;
    private boolean isPercentOutput;

    public Arm(int armTalonId){
        super();
        this.armTalon = new TalonSRX(armTalonId);
        configureTalon();

        outputsChanged = true;
        isPercentOutput = false;

        // Calibrate quadrature encoder with absolute mag encoder
        int absolutePosition = this.armTalon.getSensorCollection().getPulseWidthPosition();
        /* Mask out overflows, keep bottom 12 bits */
        absolutePosition &= 0xFFF;
        /* Set the quadrature (relative) sensor to match absolute */
        this.armTalon.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
        this.armPosition = getArmPositionAbsolute();
        BadLog.createTopic(Robot.LOG_ARM_READING, "ticks", () -> (double) this.getArmPosition());
    }

    private void configureTalon() {
        this.armTalon.configFactoryDefault();
        this.armTalon.setNeutralMode(NeutralMode.Brake);
        this.armTalon.setInverted(true);
        this.armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                kPIDLoopIdx, kTimeoutMs);

        this.armTalon.setSensorPhase(true);

        /* Config the peak and nominal outputs, 12V means full */
        this.armTalon.configNominalOutputForward(0, kTimeoutMs);
        this.armTalon.configNominalOutputReverse(0, kTimeoutMs);
        this.armTalon.configPeakOutputForward(1, kTimeoutMs);
        this.armTalon.configPeakOutputReverse(-1, kTimeoutMs);

        this.setPID(kP, kI, kD);

        this.armTalon.configAllowableClosedloopError(kPIDLoopIdx, ALLOWABLE_CLOSED_LOOP_ERROR, kTimeoutMs);

        this.armTalon.configForwardSoftLimitEnable(true);
        this.armTalon.configReverseSoftLimitEnable(true);
        this.armTalon.configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT, kTimeoutMs);
        this.armTalon.configReverseSoftLimitThreshold(REVERSE_SENSOR_LIMIT, kTimeoutMs);
        this.armTalon.set(ControlMode.PercentOutput, 0.0);
    }

    public void setArm(double armSpeed) {
        this.armSpeed = armSpeed;
        System.out.println("setArm called!");
        isPercentOutput = true;
        outputsChanged = true;
    }

    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
        isPercentOutput = false;
        outputsChanged = true;
    }

    public int getArmPositionAbsolute() {
        return armTalon.getSensorCollection().getPulseWidthPosition();
    }

    public int getArmPosition() {
        return armTalon.getSelectedSensorPosition();
    }

    public double getArmSetpoint() {
        return armPosition;
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

    public boolean isBusy() {
        if (armTalon.getControlMode() == ControlMode.Position) {
            return (armTalon.getClosedLoopError(kPIDLoopIdx) <= ALLOWABLE_CLOSED_LOOP_ERROR);
        }
        return false;
    }

    public boolean isPercentOutput() {
        return isPercentOutput;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("PID/kF", this::getkF, null);
        builder.addDoubleProperty("PID/kP", this::getkP, null);
        builder.addDoubleProperty("PID/kI", this::getkI, null);
        builder.addDoubleProperty("PID/kD", this::getkD, null);
        builder.addStringProperty("ControlMode", () -> armTalon.getControlMode().toString(), null);
        builder.addDoubleProperty("CurrentPosition", this::getArmPosition, null);
        builder.addDoubleProperty("ClosedLoop/TargetPosition", () -> (
                armTalon.getControlMode() == ControlMode.Position
                        ? armTalon.getClosedLoopTarget() : 0
        ), null);
        builder.addDoubleProperty("ClosedLoop/Error", () -> (
                armTalon.getControlMode() == ControlMode.Position
                        ? armTalon.getClosedLoopError() : 0
        ), null);
        builder.addDoubleProperty("MotorOutput", armTalon::getMotorOutputPercent, null);
        builder.addBooleanProperty("Busy", this::isBusy, null);
    }

    @Override
    public void periodic() {
        if (outputsChanged) {
            if (isPercentOutput) {
                armTalon.set(ControlMode.PercentOutput, armSpeed);
            } else {
                armTalon.set(ControlMode.Position, armPosition);
            }
            outputsChanged = false;
        }
        BadLog.publish(Robot.LOG_ARM_POS, armPosition);
    }

    @Override
    protected void initDefaultCommand() { }
}
