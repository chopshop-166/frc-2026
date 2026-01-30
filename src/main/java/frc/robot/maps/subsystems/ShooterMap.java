package frc.robot.maps.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Function;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {

    public enum ShooterPresets {
        OFF,

        CLOSE_SHOT,

        MID_SHOT,

        FAR_SHOT

    }

    public interface PresetValues extends Function<ShooterPresets, AngularVelocity> {

    }

    public SmartMotorController roller1;
    public SmartMotorController roller2;

    public PresetValues presetValues;

    public ShooterMap() {
        this(new SmartMotorController(), p -> RPM.of(0));
    }

    public ShooterMap(SmartMotorController roller, PresetValues presetValues) {
        this.roller1 = roller1;
        this.roller2 = roller2;
        this.presetValues = presetValues;

    }

    @Override
    public void updateData(Data data) {
        data.roller1.updateData(roller1);
        data.roller2.updateData(roller2);

    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller1 = new MotorControllerData(true);
        public MotorControllerData roller2 = new MotorControllerData(true);

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
        public ShooterPresets preset = ShooterPresets.OFF;
    }
}
