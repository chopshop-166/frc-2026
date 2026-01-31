package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.RollerMap;
import frc.robot.maps.subsystems.RollerMap.Data;
import frc.robot.maps.subsystems.RollerMap.RollerPresets;

public class Roller extends LoggedSubsystem<Data, RollerMap> {

    private final double ROLL_IN_SPEED = .5;
    private final double ROLL_OUT_SPEED = -.5;
    private final double WIGGLE_SPEED = .3;

    public Roller(RollerMap rollerMap) {
        super(new Data(), rollerMap);
    }

    public Command rollIn() {
        return runSafe(() -> {
            getData().roller.setpoint = RollerPresets.FORWARD;
            setPreset(RollerPresets)
        });
    }

    public Command rollOut() {
        return runSafe(() -> {
            getData().roller.setpoint = ROLL_OUT_SPEED;
        });

        
    private void setPreset(RollerPresets presets) {

        getData().preset = presets;
        getData().roller.setpoint = getMap().presetValues;
    }

   @Override
    public void safeState() {
        setPreset(RollerPresets.OFF);
    }

        
    }
}
