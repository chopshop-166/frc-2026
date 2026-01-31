package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;

import frc.robot.maps.subsystems.HoodMap;
import frc.robot.maps.subsystems.HoodMap.Data;

public class Hood extends LoggedSubsystem<Data, HoodMap> {
    final ProfiledPIDController pid;
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoubleSubscriber distanceToTargetSub = instance.getDoubleTopic("DistanceToHub").subscribe(0.0);
    double holdAngle = 0;

    public Hood(HoodMap hoodMap) {
        super(new Data(), hoodMap);
        pid = hoodMap.pid;

    }

    public Command moveToAngle(double velocity) {
        return run(() -> {
            double setpoint = pid.calculate(getHoodAngle(), new State(calcAngle(velocity), 0));
            Logger.recordOutput("ArmRotate/PID Setpoint", setpoint);
            getData().motor.setpoint = setpoint;

            Logger.recordOutput("ArmRotate/pid at goal", pid.atGoal());
            Logger.recordOutput("ArmRotate/DesiredArmVelocity", pid.getSetpoint().velocity);
            Logger.recordOutput("ArmRotate/DesiredArmPosition", pid.getSetpoint().position);
        });

    }

    public Command manualControl(DoubleSupplier joystick) {
        getData().motor.setpoint = joystick.getAsDouble();
        return null;
    }

    public double calcAngle(double velocity) {

        double GRAVITY_CONSTANT = 32.2; // in feetPerSecond
        double RADIAN_CONSTANT = 57.296; // conversion factor radians-degrees
        double ang = ((Math.asin((distanceToTargetSub.getAsDouble() * GRAVITY_CONSTANT) / velocity) / 2)
                * RADIAN_CONSTANT);
        return ang; // returns on degrees
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
