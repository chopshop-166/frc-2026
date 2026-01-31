package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    private final double TOLERANCE = 100;

    DoubleSupplier shooterSpeed;

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
    public void reset() {
        // nothing to reset
    }

    @Override
    public void safeState() {
        setPreset(ShooterPresets.OFF);
    }

}
