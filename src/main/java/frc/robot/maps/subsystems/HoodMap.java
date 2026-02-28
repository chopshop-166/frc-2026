package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.CSEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HoodMap implements LoggableMap<HoodMap.Data> {

    public SmartMotorController motor;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;

    public HoodMap() {
        this(new SmartMotorController(),
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
                new ValueRange(0, 0));
    }

    public HoodMap(SmartMotorController motor,
            ProfiledPIDController pid, ValueRange hardLimits) {
        this.motor = motor;
        this.pid = pid;
        this.hardLimits = hardLimits;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
    }
}
