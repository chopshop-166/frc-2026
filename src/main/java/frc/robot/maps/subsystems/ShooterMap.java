package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {

    public enum ShooterPresets {
        INTAKE,

        OUTTAKE,

        CLOSE_SHOT,

        MID_SHOT,

        FAR_SHOT,

        OFF
    }

    public interface PresetValues extends ToDoubleFunction<ShooterPresets> {

    }

    public SmartMotorController roller;
    public PresetValues presetValues;
    public IEncoder encoder;

    public ShooterMap() {
        this(new SmartMotorController(), p -> Double.NaN, new MockEncoder());
    }

    public ShooterMap(SmartMotorController roller, PresetValues presetValues, IEncoder encoder) {
        this.roller = roller;
        this.presetValues = presetValues;
        this.encoder = encoder;

    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(roller);
        data.speedRotationsPerMinute = encoder.getRate();

    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData(true);

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
        public double speedRotationsPerMinute;
        public ShooterPresets preset = ShooterPresets.OFF;
    }
}
