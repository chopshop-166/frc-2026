package frc.robot.maps.subsystems;

import static edu.wpi.first.units.Units.Degrees;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DeployerMap implements LoggableMap<DeployerMap.Data> {

    public enum DeployerPresets {

        OFF,

        OUT,

        IN,

        HOLD
    }

    public interface PresetValue extends Function<DeployerPresets, Angle> {

    }

    public SmartMotorController motor;
    public final DutyCycleEncoder encoder;
    public final PresetValue deployerPreset;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;
    public final ArmFeedforward armFeedforward;

    public DeployerMap() {
        this(new SmartMotorController(), new DutyCycleEncoder(0), p -> Degrees.of(0),
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)), new ValueRange(0, 0), new ValueRange(0, 0),
                new ArmFeedforward(0, 0, 0));
    }

    public DeployerMap(SmartMotorController motor, DutyCycleEncoder encoder, PresetValue deployerPreset,
            ProfiledPIDController pid, ValueRange hardLimits, ValueRange softLimits, ArmFeedforward armFeedforward) {
        this.motor = motor;
        this.encoder = encoder;
        this.deployerPreset = deployerPreset;
        this.pid = pid;
        this.softLimits = softLimits;
        this.hardLimits = hardLimits;
        this.armFeedforward = armFeedforward;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.rotationAbsAngleDegrees = encoder.get();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
        public DeployerPresets preset = DeployerPresets.OFF;
    }

}
