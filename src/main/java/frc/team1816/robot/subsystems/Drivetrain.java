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
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;
    public static final double kDriveVoltageRampRate = 0.0;

    private double leftPower, rightPower = 0;

    public Drivetrain(int leftMainID, int leftSlaveOneID, int leftSlaveTwoID, int rightMainID, int rightSlaveOneID,
                      int rightSlaveTwoID) {
        super();

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
    }

    public void setDrivetrain(double leftPower, double rightPower){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    public double getLeftPower(){
        return leftPower;
    }

    public double getRightPower(){
        return rightPower;
    }

    public void update() {
        this.leftMain.set(ControlMode.PercentOutput, leftPower);
        this.rightMain.set(ControlMode.PercentOutput, rightPower);
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
