package frc.team1816.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {

    private DoubleSolenoid solenoid;

    public Shooter(int pcmId, int solenoid1, int solenoid2){
        this.solenoid = new DoubleSolenoid(pcmId, solenoid1, solenoid2);
        this.solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void shoot(){
        this.solenoid.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        this.solenoid.set(DoubleSolenoid.Value.kForward);
        Timer.delay(0.5);
        this.solenoid.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        this.solenoid.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    protected void initDefaultCommand() { }
}
