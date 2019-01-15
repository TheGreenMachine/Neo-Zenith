package frc.team1816.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team1816.robot.Robot;

public class Arm extends Subsystem {
    private TalonSRX armTalon;

    private double armSpeed;
    private final double LOWER_THRESHOLD = 0.164;
    private final double UPPER_THRESHOLD = 0.363;

    private AnalogPotentiometer potentiometer;
    private double armPosition;

    public Arm(int armTalon, int potentiometer){
        super();
        this.armTalon = new TalonSRX(armTalon);
        this.armTalon.setNeutralMode(NeutralMode.Brake);
        this.armTalon.setInverted(true);
        this.potentiometer = new AnalogPotentiometer(potentiometer);
    }

    public void setArm(double armSpeed){
        if (((getArmPos() < LOWER_THRESHOLD) && (armSpeed < 0)) || ((getArmPos() > UPPER_THRESHOLD) && (armSpeed > 0))){
            this.armSpeed = 0;
        } else {
            this.armSpeed = armSpeed;
        }
        this.armTalon.set(ControlMode.PercentOutput, this.armSpeed);
    }

    public double getArmPos() {
        return armPosition;
    }


    @Override
    public void periodic() {
        armPosition = potentiometer.get();
        BadLog.publish(Robot.LOG_ARM_POS, armPosition);
    }

    @Override
    protected void initDefaultCommand() { }
}
