package frc.team1816.robot;

public class Controls {
    private static Controls instance;

    public Controls() {

    }

    public static Controls getInstance(){
        if (instance == null){
            instance = new Controls();
        }
        return instance;
    }
}
