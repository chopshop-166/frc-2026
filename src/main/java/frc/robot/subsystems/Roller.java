package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.states.LinearDirection;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.RollerMap;
import frc.robot.maps.subsystems.RollerMap.Data;
import frc.robot.maps.subsystems.RollerMap.RollerPresets;

public class Roller extends LoggedSubsystem<Data, RollerMap> {

    public Roller(RollerMap rollerMap) {
        super(new Data(), rollerMap);
    }

    public Command rollIn() {
        return runOnce(() -> {
            setPreset(RollerPresets.FORWARD);
        });
    }

    public Command rollOut() {
        return runOnce(() -> {
            setPreset(RollerPresets.REVERSE);
        });
    }

    public Command wiggle(LinearDirection direction) {
        if (direction == LinearDirection.FORWARD) {
            return runOnce(() -> {
                setPreset(RollerPresets.FORWARD_WIGGLE);
            }).andThen(Commands.waitTime(Time.ofRelativeUnits(.5, Seconds))).andThen(safeStateCmd());
        } else {
            return runOnce(() -> {
                setPreset(RollerPresets.BACKWARDS_WIGGLE);
            }).andThen(Commands.waitTime(Time.ofRelativeUnits(.5, Seconds))).andThen(safeStateCmd());
        }

    }

    private void setPreset(RollerPresets presets) {

        getData().preset = presets;
        getData().roller.setpoint = getMap().presetValues.applyAsDouble(getData().preset);
    }

    @Override
    public void safeState() {
        setPreset(RollerPresets.OFF);
    }

    @Override
    public void reset() {
        // nothing to reset
    }

}
