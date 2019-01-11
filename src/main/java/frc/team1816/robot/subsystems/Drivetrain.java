package frc.team1816.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.beans.BeanDescriptor;
import java.beans.PropertyDescriptor;
import java.util.Enumeration;

public class Drivetrain extends Subsystem {
    private TalonSRX leftMain, leftSlaveOne, leftSlaveTwo, rightMain, rightSlaveOne, rightSlaveTwo;

    private double leftPower, rightPower = 0;
    private static double kP = 0.4;
    private static double kI = 0.0;
    private static double kD = 0.0;
//    private static double kF = 0.646;
    private static double kF = 0.146;
    private static int izone = 0;

    double lastLeftPower;
    double lastRightPower;

    public Drivetrain(int leftMain, int leftSlaveOne, int leftSlaveTwo, int rightMain, int rightSlaveOne,
                      int rightSlaveTwo) {
        super();

        this.leftMain = new TalonSRX(leftMain);
        this.leftSlaveOne = new TalonSRX(leftSlaveOne);
        this.leftSlaveTwo = new TalonSRX(leftSlaveTwo);

        this.rightMain = new TalonSRX(rightMain);
        this.rightSlaveOne = new TalonSRX(rightSlaveOne);
        this.rightSlaveTwo = new TalonSRX(rightSlaveTwo);

        this.leftMain.setInverted(true);
        this.leftSlaveOne.setInverted(true);
        this.leftSlaveTwo.setInverted(true);

        this.leftMain.setNeutralMode(NeutralMode.Brake);
        this.leftSlaveOne.setNeutralMode(NeutralMode.Brake);
        this.leftSlaveTwo.setNeutralMode(NeutralMode.Brake);

        this.rightMain.setNeutralMode(NeutralMode.Brake);
        this.rightSlaveOne.setNeutralMode(NeutralMode.Brake);
        this.rightSlaveTwo.setNeutralMode(NeutralMode.Brake);

        this.leftSlaveOne.set(ControlMode.Follower, leftMain);
        this.leftSlaveTwo.set(ControlMode.Follower, leftMain);

        this.rightSlaveOne.set(ControlMode.Follower, rightMain);
        this.rightSlaveTwo.set(ControlMode.Follower, rightMain);

        this.rightMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        this.leftMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

        this.rightMain.configNominalOutputForward(0, 10);
        this.rightMain.configNominalOutputReverse(0, 10);
        this.rightMain.configPeakOutputForward(1, 10);
        this.rightMain.configPeakOutputReverse(-1, 10);

        this.leftMain.configNominalOutputForward(0, 10);
        this.leftMain.configNominalOutputReverse(0, 10);
        this.leftMain.configPeakOutputForward(1, 10);
        this.leftMain.configPeakOutputReverse(-1, 10);

        this.leftMain.config_kP(0, kP, 20);
        this.leftMain.config_kI(0, kI, 20);
        this.leftMain.config_kD(0, kD, 20);
        this.leftMain.config_kF(0, kF, 20);
        this.leftMain.config_IntegralZone(0, izone, 20);

        this.rightMain.config_kP(0, kP, 20);
        this.rightMain.config_kI(0, kI, 20);
        this.rightMain.config_kD(0, kD, 20);
        this.rightMain.config_kF(0, kF, 20);
        this.rightMain.config_IntegralZone(0, izone, 20);

        this.leftMain.selectProfileSlot(0,0);
        this.rightMain.selectProfileSlot(0,0);

        this.rightMain.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms,0);
        this.leftMain.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms,0);

        this.rightMain.configVelocityMeasurementWindow(8,0);
        this.leftMain.configVelocityMeasurementWindow(8,0);
    }

    public void setDrivetrain(double leftPower, double rightPower){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        System.out.printf("--- leftPower=%5.2f, rightPower=%5.2f\n", leftPower, rightPower);
        lastLeftPower = leftPower;
        lastRightPower = rightPower;
//        update();
    }

    @Override
    public void periodic() {
        int leftVel = this.leftMain.getSelectedSensorVelocity(0);
        int rightVel = this.rightMain.getSelectedSensorVelocity(0);

        leftMain.getSensorCollection();

        System.out.printf("Left Velocity: %5d,  Right Velocity: %5d\n", leftVel,  rightVel);
        this.leftMain.set(ControlMode.Velocity, leftPower * 1023.0);
        this.rightMain.set(ControlMode.Velocity, rightPower * 1023.0);
    }

    @Override
    protected void initDefaultCommand() { }

    void print(SensorCollection sensorCollection) {

        BeanDescriptor bean = new BeanDescriptor(SensorCollection.class);
        Enumeration<String> names = bean.attributeNames();
        while(names.hasMoreElements()) {
            String name = names.nextElement();
            try {
                PropertyDescriptor pd = new PropertyDescriptor(name, SensorCollection.class);
                Object obj = pd.getReadMethod().invoke(sensorCollection);

                System.out.printf(" [%s=%");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
