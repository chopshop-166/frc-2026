package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
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

    DoubleSupplier shooterSpeed;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoublePublisher shooterVelocityFPSPub = instance.getDoubleTopic("Shooter/Linear Velocity").publish();

    public Shooter(ShooterMap shooterMap) {
        super(new Data(), shooterMap);

    }

    public Command shoot(ShooterPresets presetSpeed) {
        Debouncer debouncer = new Debouncer(.2, DebounceType.kBoth);

        return runOnce(() -> {

            setPreset(presetSpeed);

        }).andThen(Commands.waitUntil(() -> debouncer
                .calculate(Math.abs(getData().flywheel.velocity - getData().flywheel.setpoint) < TOLERANCE)));

    }

    private void setPreset(ShooterPresets presets) {

        getData().preset = presets;
        getData().flywheel.setpoint = getMap().presetValues.apply(getData().preset).in(RPM);

    }

    public double getShooterSpeed() {
        return getMap().flywheel.getEncoder().getRate();
    }

    @Override
    public void periodic() {
        super.periodic();
        // converting angular velocity to linear velocity
        shooterVelocityFPSPub.set((getData().flywheel.velocity * Math.PI * 4) / 60);
        Logger.recordOutput("Shooter/PID Error", getData().flywheel.setpoint - getData().flywheel.velocity);

    }

    @Override
    public void safeState() {
        setPreset(ShooterPresets.OFF);
    }

}
