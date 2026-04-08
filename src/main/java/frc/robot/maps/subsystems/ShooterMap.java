package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {

    public enum ShooterPresets {
        // No preset
        OFF,
        // Shoot up close
        CLOSE_SHOT,
        // Mid range shot
        MID_SHOT,
        // Shoot for distance
        FAR_SHOT,
        // Get the value from NT
        NETWORK_TABLES,
        // Get multiple tunable values from NT
        NETWORK_TABLES_AUTO,

        AUTO_SPEED

    }

    public interface PresetValues extends ToDoubleFunction<ShooterPresets> {

    }

    public SmartMotorController flywheel;

    public PresetValues presetValues;

    public ShooterMap() {
        this(new SmartMotorController(), p -> 0);
    }

    public ShooterMap(SmartMotorController flywheel, PresetValues presetValues) {
        this.flywheel = flywheel;
        this.presetValues = presetValues;

    }

    @Override
    public void updateData(Data data) {
        data.flywheel.updateData(flywheel);

    }

    public static class Data extends DataWrapper {
        public MotorControllerData flywheel = new MotorControllerData(true);

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
        public ShooterPresets preset = ShooterPresets.OFF;
    }
}
