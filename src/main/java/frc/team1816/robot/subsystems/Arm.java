package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team1816.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Arm extends Subsystem {
    private TalonSRX armTalon;

    private double armSpeed;
    private final double LOWER_THRESHOLD = 0.164;
    private final double UPPER_THRESHOLD = 0.363;

    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 30;

    private double kF = 0;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

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
        this.armTalon.configPeakOutputForward(0.8, kTimeoutMs);
        this.armTalon.configPeakOutputReverse(-0.8, kTimeoutMs);

        this.armTalon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        this.armTalon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        this.armTalon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        this.armTalon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

        this.armTalon.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

        this.armTalon.configForwardSoftLimitEnable(true);
        this.armTalon.configReverseSoftLimitEnable(true);
        this.armTalon.configForwardSoftLimitThreshold(1475, kTimeoutMs);
        this.armTalon.configReverseSoftLimitThreshold(915, kTimeoutMs);

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
        armTalon.set(ControlMode.Position, armPosition * 480 * 4096);
    }

    public double getArmPosition() {
        return armTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getSetArmPosition() {
        return armSetpoint;
    }

    public double getArmPos() {
        return armPosition;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
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
        builder.addDoubleProperty("Arm/kP", this::getkP, this::setkP);
        builder.addDoubleProperty("Arm/kI", this::getkI, this::setkI);
        builder.addDoubleProperty("Arm/kD", this::getkD, this::setkD);
        builder.addDoubleProperty("Arm/ArmPosition/real", this::getArmPosition, null);
    }

    @Override
    public void periodic() {
        armPosition = potentiometer.get();
        BadLog.publish(Robot.LOG_ARM_POS, armPosition);
    }

    @Override
    protected void initDefaultCommand() { }
}
