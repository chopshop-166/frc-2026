package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import edu.wpi.first.units.Units;

import frc.robot.maps.subsystems.HoodMap;
import frc.robot.maps.subsystems.HoodMap.Data;

public class Hood extends LoggedSubsystem<Data, HoodMap> {
    final ProfiledPIDController pid;
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoubleSubscriber distanceToTargetSub = instance.getDoubleTopic("Drive/Distance To Hub Ft").subscribe(0.0);
    DoubleSubscriber ShooterLinearVelocity = instance.getDoubleTopic("Shooter/Linear Velocity").subscribe(0.0);
    double holdAngle = 0;

    public Hood(HoodMap hoodMap) {
        super(new Data(), hoodMap);
        pid = hoodMap.pid;

    }

    public Command moveToAngle(double velocity) {
        return run(() -> {
            double setpoint = pid.calculate(getHoodAngle(), new State(calcAngle(velocity), 0));
            Logger.recordOutput("Hood/PID Setpoint", setpoint);
            getData().motor.setpoint = setpoint;

            Logger.recordOutput("Hood/pid at goal", pid.atGoal());
            Logger.recordOutput("Hood/Desired Hood Velocity", pid.getSetpoint().velocity);
            Logger.recordOutput("Hood/Desired Hood Position", pid.getSetpoint().position);
            Logger.recordOutput("Hood/Position Error", pid.getPositionError());
        });

    }

    public Command manualControl(DoubleSupplier joystick) {
        return run(() -> {
            getData().motor.setpoint = joystick.getAsDouble();
        });
    }

    public Double calcAngle(double velocity) {
        final Double targetDistanceInFeet = distanceToTargetSub.get();
        final Double ShooterLinearVelocityInFPS = ShooterLinearVelocity.get();
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
        return getData().rotationAbsAngleDegrees;
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }
}
