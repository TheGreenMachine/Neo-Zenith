package frc.team1816.robot;

import frc.team1816.robot.subsystems.Arm;
import frc.team1816.robot.subsystems.Drivetrain;
import frc.team1816.robot.subsystems.Intake;
import frc.team1816.robot.subsystems.Shooter;

public class Components {
    private static Components instance;

    public Drivetrain drivetrain;
    public Shooter shooter;
    public Arm arm;
    public Intake intake;

    //Drivetrain Talon Id's
    private final int LEFT_MAIN = 1;
    private final int LEFT_SLAVE_ONE = 2;
    private final int LEFT_SLAVE_TWO = 3;
    private final int RIGHT_MAIN = 5;
    private final int RIGHT_SLAVE_ONE = 6;
    private final int RIGHT_SLAVE_TWO = 7;

    //Arm and Intake
    private final int ARM_TALON = 4;
    private final int ANALOG_POTENTIOMETER_ID = 0;
    private final int INTAKE_TALON = 8;

    //Shooter Constants
    private final int PCM_ID = 10;
    private final int SOLENOID_ID_1 = 1;
    private final int SOLENOID_ID_2 = 2;

    //Mode Constants
//    public final double ARM_SPEED = 0.5;
//
//    public final double INTAKE_SPEED = 1;

    public Components(){
        this.drivetrain = new Drivetrain(LEFT_MAIN, LEFT_SLAVE_ONE, LEFT_SLAVE_TWO, RIGHT_MAIN, RIGHT_SLAVE_ONE,
                RIGHT_SLAVE_TWO);
        this.shooter = new Shooter(PCM_ID, SOLENOID_ID_1, SOLENOID_ID_2);
        this.arm = new Arm(ARM_TALON, ANALOG_POTENTIOMETER_ID);
        this.intake = new Intake(INTAKE_TALON);
    }

    public static Components getInstance(){
        if (instance == null){
            instance = new Components();
        }
        return instance;
    }


}
