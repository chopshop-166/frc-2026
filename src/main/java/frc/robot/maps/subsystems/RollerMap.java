package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import frc.robot.maps.subsystems.ShooterMap.PresetValues;

public class RollerMap implements LoggableMap<RollerMap.Data> {
    SmartMotorController roller;

    public enum RollerPresets {
        FORWARD,

        REVERSE,

        WIGGLE,

        OFF

    }

    public PresetValues presetValues;

    public RollerMap() {
        this(new SmartMotorController());
    }

    public RollerMap(SmartMotorController roller) {
        this.roller = roller;
        this.presetValues = presetValues;
    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(roller);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData();
    }
}
