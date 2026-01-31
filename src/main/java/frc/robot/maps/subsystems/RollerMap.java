package frc.robot.maps.subsystems;

import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.ToDoubleFunction;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.maps.subsystems.ShooterMap.PresetValues;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;

public class RollerMap implements LoggableMap<RollerMap.Data> {

    SmartMotorController roller;

    public enum RollerPresets {
        FORWARD,

        REVERSE,

        FORWARD_WIGGLE,

        BACKWARDS_WIGGLE,

        OFF

    }

    public interface PresetValues extends ToDoubleFunction<RollerPresets> {

    }

    public PresetValues presetValues;

    public RollerMap() {
        this(new SmartMotorController(), p -> Double.NaN);
    }

    public RollerMap(SmartMotorController roller, PresetValues presetValues) {
        this.roller = roller;
        this.presetValues = presetValues;
    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(roller);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData();
        public RollerPresets preset = RollerPresets.OFF;
    }
}
