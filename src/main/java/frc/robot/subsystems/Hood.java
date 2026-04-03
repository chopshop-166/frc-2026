package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import frc.robot.maps.subsystems.HoodMap;
import frc.robot.maps.subsystems.HoodMap.Data;
import frc.robot.maps.subsystems.HoodMap.HoodPresets;

public class Hood extends LoggedSubsystem<Data, HoodMap> {
    final ProfiledPIDController pid;
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoubleSubscriber distanceToTargetSub = instance.getDoubleTopic("Drive/Distance To Hub Ft").subscribe(0.0);
    DoubleSubscriber shooterLeftLinearVelocity = instance.getDoubleTopic("Shooter/Left/Linear Velocity").subscribe(0.0);
    DoubleSubscriber shooterRightLinearVelocity = instance.getDoubleTopic("Shooter/Right/Linear Velocity")
            .subscribe(0.0);

    public Hood(HoodMap hoodMap) {
        super(new Data(), hoodMap);
        pid = hoodMap.pid;

    }

    public Command moveToAngle(HoodPresets preset) {
        return runOnce(() -> {
            pid.reset(getHoodAngle());
            getData().preset = preset;
        });
    }

    public Command manualControl(DoubleSupplier joystick) {
        return run(() -> {
            getData().motor.setpoint = limits(joystick.getAsDouble());
        });
    }

    public Command zero() {
        return runOnce(() -> {
            getMap().motor.getEncoder().reset();
        });
    }

    // public Command autoZero() {
    // return startSafe(()-> {
    // getMap().motor.resetValidators();
    // getData().preset = HoodPresets.ZEROING;
    // });
    // }

    public Double calcAngle(double velocity) {
        final Double targetDistanceInFeet = distanceToTargetSub.get();
        final Double ShooterLinearVelocityInFPS = (shooterLeftLinearVelocity.get() + shooterRightLinearVelocity.get())
                / 2;
        final Double GRAVITY_CONSTANT_IN_FPS = 32.2;
        final Double RADIANS_DEGREE_CONVERSION_FACTOR = 57.2958;
        final Double targetVelocitySqrInFPS = (ShooterLinearVelocityInFPS * ShooterLinearVelocityInFPS);

        Double ang = ((Math.asin((targetDistanceInFeet * GRAVITY_CONSTANT_IN_FPS) / (targetVelocitySqrInFPS)) / 2)
                * RADIANS_DEGREE_CONVERSION_FACTOR);
        return ang; // in degrees
    }

    private double limits(double speed) {
        double angle = getHoodAngle();
        speed = getMap().hardLimits.filterSpeed(angle, speed);
        return speed;
    }

    private double getHoodAngle() {
        return getData().motor.distance;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (getData().preset != HoodPresets.OFF) {
            double targetAngle = getMap().hoodPreset.applyAsDouble(getData().preset);
            Logger.recordOutput("Hood/TargetAngle", targetAngle);
            double setpoint = pid.calculate(getHoodAngle(), new State(targetAngle, 0));
            Logger.recordOutput("Hood/PID Setpoint", setpoint);
            setpoint += getMap().armFeedforward.calculate(
                    pid.getSetpoint().position,
                    pid.getSetpoint().velocity);
            Logger.recordOutput("Hood/FF Setpoint", setpoint);
            getData().motor.setpoint = setpoint;
            Logger.recordOutput("Hood/pid at goal", pid.atGoal());
            Logger.recordOutput("Hood/Desired Hood Velocity", pid.getSetpoint().velocity);
            Logger.recordOutput("Hood/Desired Hood Position", pid.getSetpoint().position);
            Logger.recordOutput("Hood/Position Error", pid.getPositionError());
            Logger.recordOutput("Hood/CurrentAngleRadians", getHoodAngle());
        } else {
            getData().motor.setpoint = 0;
        }
    }

    @Override
    public void safeState() {
        getData().preset = HoodPresets.OFF;
    }
}
