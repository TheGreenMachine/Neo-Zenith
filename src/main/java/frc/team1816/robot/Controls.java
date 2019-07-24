package frc.team1816.robot;

import com.edinarobotics.utils.gamepad.FilteredGamepad;
import com.edinarobotics.utils.gamepad.Gamepad;
import com.edinarobotics.utils.gamepad.gamepadfilters.DeadzoneFilter;
import com.edinarobotics.utils.gamepad.gamepadfilters.GamepadFilter;
import com.edinarobotics.utils.gamepad.gamepadfilters.GamepadFilterSet;
import com.edinarobotics.utils.gamepad.gamepadfilters.PowerFilter;
import frc.team1816.robot.commands.*;

import frc.team1816.robot.commands.*;
import frc.team1816.robot.subsystems.Arm;

import java.util.ArrayList;
import java.util.List;

public class Controls {
    private static Controls instance;

    public Gamepad gamepadDriver;

    public Controls() {
        List<GamepadFilter> gamepadFilters = new ArrayList<>();
        gamepadFilters.add(new DeadzoneFilter(0.05));
        gamepadFilters.add(new PowerFilter(2));
        GamepadFilterSet gamepadDriverFilterSet = new GamepadFilterSet(gamepadFilters);

        gamepadDriver = new FilteredGamepad(0,gamepadDriverFilterSet);
        gamepadDriver.diamondUp().whileHeld(new DriveToHatchCommand(0.30));
    }

    public static Controls getInstance(){
        if (instance == null){
            instance = new Controls();
        }
        return instance;
    }

    public double getDriveSpeed() {
        return gamepadDriver.getLeftY();
    }

    public double getDriveTurn() {
        return gamepadDriver.getRightX();
    }

    public boolean isShoot() {
        return gamepadDriver.diamondDown().get();
    }
}
