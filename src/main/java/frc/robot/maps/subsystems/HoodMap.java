package frc.robot.maps.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Function;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;

public class HoodMap implements LoggableMap<HoodMap.Data> {

    public enum HoodPresets {
        OFF,

        CLOSE,

        MID,

        FAR,

        NETWORK_TABLES;
    }

    public interface PresetValue extends Function<HoodPresets, Angle> {

    }

    public SmartMotorController motor;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;
    public final ArmFeedforward armFeedforward;
    public final PresetValue hoodPreset;

    public HoodMap() {
        this(new SmartMotorController(),
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
                new ValueRange(0, 0),
                new ArmFeedforward(0, 0, 0), p -> Radians.of(0));
    }

    public HoodMap(SmartMotorController motor,
            ProfiledPIDController pid, ValueRange hardLimits, ArmFeedforward armFeedforward, PresetValue hoodPreset) {
        this.motor = motor;
        this.pid = pid;
        this.hardLimits = hardLimits;
        this.armFeedforward = armFeedforward;
        this.hoodPreset = hoodPreset;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public HoodPresets preset = HoodPresets.OFF;

    }
}
