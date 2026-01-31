package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import frc.robot.maps.subsystems.DeployerMap.PresetValue;

public class HoodMap implements LoggableMap<HoodMap.Data> {

    public SmartMotorController motor;
    public final DutyCycleEncoder encoder;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;

    public HoodMap() {
        this(new SmartMotorController(), new DutyCycleEncoder(0),
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
                new ValueRange(0, 0));
    }

    public HoodMap(SmartMotorController motor, DutyCycleEncoder encoder,
            ProfiledPIDController pid, ValueRange hardLimits) {
        this.motor = motor;
        this.encoder = encoder;
        this.pid = pid;
        this.hardLimits = hardLimits;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.rotationAbsAngleDegrees = encoder.get();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
    }
}
