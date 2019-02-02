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

    private static final int FORWARD_SENSOR_LIMIT = 1475;
    private static final int REVERSE_SENSOR_LIMIT = 915;

    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 30;

    private double kF = 0;
    private double kP = 5.0;
    private double kI = 0;
    private double kD = 1.0;

    private double armPosition;
    private double armSpeed;

    private boolean outputsChanged = true;
    private boolean isPercentOutput = false;

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

        this.armTalon.configAllowableClosedloopError(kPIDLoopIdx, 50, kTimeoutMs);

        this.armTalon.configForwardSoftLimitEnable(true);
        this.armTalon.configReverseSoftLimitEnable(true);
        this.armTalon.configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT, kTimeoutMs);
        this.armTalon.configReverseSoftLimitThreshold(REVERSE_SENSOR_LIMIT, kTimeoutMs);
    }

    public void setArm(double armSpeed){
        this.armSpeed = armSpeed;
        isPercentOutput = true;
        outputsChanged = true;
    }

    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
        isPercentOutput = false;
        outputsChanged = true;
    }

    public double getArmPositionAbsolute() {
        return armTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getArmPosition() {
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm/PID/kF", this::getkF, null);
        builder.addDoubleProperty("Arm/PID/kP", this::getkP, null);
        builder.addDoubleProperty("Arm/PID/kI", this::getkI, null);
        builder.addDoubleProperty("Arm/PID/kD", this::getkD, null);
        builder.addStringProperty("Arm/ControlMode", () -> armTalon.getControlMode().toString(), null);
        builder.addDoubleProperty("Arm/CurrentPosition", this::getArmPosition, null);
        builder.addDoubleProperty("Arm/TargetPosition", armTalon::getClosedLoopTarget, null);
        builder.addDoubleProperty("Arm/ClosedLoopError", () -> armTalon.getClosedLoopError(0), null);
        builder.addDoubleProperty("Arm/MotorOutput", armTalon::getMotorOutputPercent, null);
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
