package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    private final double SHOOT_SPEED = 1.0;
    private final double GRAB_SPEED = 0.5;
    private final double RELEASE_SPEED = -0.5;
    private final double TOLERANCE = 100;

    DoubleSupplier shooterSpeed;

    public Shooter(ShooterMap shooterMap) {
        super(new Data(), shooterMap);
    }

    public Command spinIn() {
        return runSafe(() -> {

            getData().preset = ShooterPresets.INTAKE;
            getData().roller.setpoint = getMap().presetValues.applyAsDouble(getData().preset);
        });
    }

    public Command shoot() {
        Debouncer debouncer = new Debouncer(.2, DebounceType.kBoth);

        return runOnce(() -> {

            getData().preset = ShooterPresets.MID_SHOT;
            getData().roller.setpoint = getMap().presetValues.applyAsDouble(getData().preset);

        }).andThen(run(() -> {
        }).until(
                () -> debouncer
                        .calculate(Math.abs(getData().roller.velocity - getData().roller.setpoint) < TOLERANCE)));
    }

    public Command spinOut() {
        return runSafe(() -> {
            getData().preset = ShooterPresets.OUTTAKE;
            getData().roller.setpoint = getMap().presetValues.applyAsDouble(getData().preset);

        });
    }

    public double getShooterSpeed() {
        return getData().speedRotationsPerMinute;
    }

    @Override
    public void reset() {
        // nothing to reset
    }

    @Override
    public void safeState() {
        getData().roller.setpoint = 0;
        getData().preset = ShooterPresets.OFF;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

}
