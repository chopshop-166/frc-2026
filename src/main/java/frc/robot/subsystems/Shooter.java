package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    private final double TOLERANCE = 100;

    private final String implName;
    DoubleSupplier shooterSpeed;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoublePublisher shooterVelocityFPSPub;

    public Shooter(ShooterMap shooterMap, String name) {
        super(new Data(), shooterMap);
        this.implName = name;
        shooterVelocityFPSPub = instance.getDoubleTopic(getName() + "/Linear Velocity").publish();
    }

    @Override
    public String getName() {
        return implName;
    }

    public Command spinUp(ShooterPresets presetSpeed) {
        Debouncer debouncer = new Debouncer(.2, DebounceType.kBoth);

        return runOnce(() -> {
            getData().preset = presetSpeed;
        }).andThen(Commands.waitUntil(() -> debouncer.calculate(Math.abs(getError()) < TOLERANCE)));

    }

    public double getShooterSpeed() {
        return getMap().flywheel.getEncoder().getRate();
    }

    @Override
    public void periodic() {
        super.periodic();
        // converting angular velocity to linear velocity
        shooterVelocityFPSPub.set((getData().flywheel.velocity * Math.PI * 4) / 60);
        getData().flywheel.setpoint = getMap().presetValues.applyAsDouble(getData().preset);
        Logger.recordOutput(getName() + "/PID Error", getError());

    }

    @Override
    public void safeState() {
        getData().preset = ShooterPresets.OFF;
    }

    private double getError() {
        return getData().flywheel.setpoint - getData().flywheel.velocity;
    }

}
