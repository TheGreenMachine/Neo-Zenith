package frc.team1816.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem {
    private TalonSRX arm;

    private double armSpeed;

    //failsafe options
    private AnalogPotentiometer potentiometer;

    public Arm(int arm, int pcmId, int solenoid, int potentiometer){
        super();
        this.arm = new TalonSRX(arm);
        this.potentiometer = new AnalogPotentiometer(potentiometer);
    }

    public void setArm(double armSpeed){
        this.armSpeed = -armSpeed * 0.75;
        this.arm.set(ControlMode.Velocity, armSpeed);
    }

    @Override
    protected void initDefaultCommand() { }
}
