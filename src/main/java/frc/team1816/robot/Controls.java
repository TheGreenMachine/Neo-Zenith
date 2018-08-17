package frc.team1816.robot;

import com.edinarobotics.utils.gamepad.FilteredGamepad;
import com.edinarobotics.utils.gamepad.Gamepad;
import com.edinarobotics.utils.gamepad.gamepadfilters.DeadzoneFilter;
import com.edinarobotics.utils.gamepad.gamepadfilters.GamepadFilter;
import com.edinarobotics.utils.gamepad.gamepadfilters.GamepadFilterSet;
import com.edinarobotics.utils.gamepad.gamepadfilters.PowerFilter;

import java.util.ArrayList;
import java.util.List;

public class Controls {
    private static Controls instance;

    public Gamepad gamepadDriver, gamepadOverride;

    public Controls() {
        List<GamepadFilter> gamepadFilters = new ArrayList<>();
        gamepadFilters.add(new DeadzoneFilter(0.05));
        gamepadFilters.add(new PowerFilter(1)); //Linear filter for demo
        GamepadFilterSet gamepadDriverFilterSet = new GamepadFilterSet(gamepadFilters);

        gamepadDriver = new FilteredGamepad(0,gamepadDriverFilterSet);
        gamepadOverride = new Gamepad(1);

    }

    public static Controls getInstance(){
        if (instance == null){
            instance = new Controls();
        }
        return instance;
    }
}
