package frc.team1816.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {
    private TalonSRX collector;

    public Intake(int collector){
        super();
        this.collector = new TalonSRX(collector);
    }

    public void setIntake(double velocity){
        this.collector.set(ControlMode.Velocity, velocity);
    }


    @Override
    protected void initDefaultCommand() {
    }
}
