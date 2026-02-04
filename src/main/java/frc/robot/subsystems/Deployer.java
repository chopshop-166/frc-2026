package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.DeployerMap;
import frc.robot.maps.subsystems.DeployerMap.Data;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;

public class Deployer extends LoggedSubsystem<Data, DeployerMap> {

    final ProfiledPIDController pid;
    private final double RAISE_SPEED_COEF = 0.5;
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final double SLOW_DOWN_COEF = 0.3;
    double holdAngle = 0;

    DoubleSupplier deployerSpeed;

    public Deployer(DeployerMap deployerMap, DoubleSupplier deployerSpeed) {
        super(new Data(), deployerMap);
        pid = deployerMap.pid;
        this.deployerSpeed = deployerSpeed;

    }

    public Command moveTo(DeployerPresets level) {
        Debouncer debouncer = new Debouncer(.2, DebounceType.kBoth);

        return runOnce(() -> {
            getData().preset = level;
            pid.reset(getDeployerAngle(), 0.0);
        }).andThen(Commands.waitUntil(() -> debouncer.calculate(pid.atGoal()))).withName("Move To Set Angle");
    }

    public Command moveToNonOwning(DeployerPresets level) {
        return Commands.runOnce(() -> {
            getData().preset = level;
            pid.reset(getDeployerAngle(), 0.0);
        }).withName("Move To Set Angle (Non-Owning)");
    }

    public Command hold() {
        return runOnce(() -> {
            holdAngle = getDeployerAngle();
            getData().preset = DeployerPresets.HOLD;
        }).withName("Hold");
    }

    private double limits(double speed) {
        double height = getDeployerAngle();
        speed = getMap().hardLimits.filterSpeed(height, speed);
        speed = getMap().softLimits.scaleSpeed(height, speed, SLOW_DOWN_COEF);
        return speed;
    }

    private double getDeployerAngle() {
        return getData().rotationAbsAngleDegrees;
    }

    @Override
    public void periodic() {
        super.periodic();

        double speed = deployerSpeed.getAsDouble();

        if (Math.abs(speed) > 0) {
            getData().preset = DeployerPresets.OFF;

            double speedCoef = RAISE_SPEED_COEF;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }

            getData().motor.setpoint = (limits(speed * speedCoef));

        } else if (getData().preset != DeployerPresets.OFF) {
            double targetHeight = getData().preset == DeployerPresets.HOLD ? holdAngle
                    : getMap().deployerPreset.apply(getData().preset).in(Degrees);
            double setpoint = pid.calculate(getDeployerAngle(), new State(targetHeight, 0));
            Logger.recordOutput("Deployer/PID Setpoint", setpoint);

            setpoint += getMap().armFeedforward.calculate(
                    pid.getSetpoint().position,
                    pid.getSetpoint().velocity);
            Logger.recordOutput("Deployer/PID+FF Setpoint", setpoint);
            getData().motor.setpoint = setpoint;
        } else {
            getData().motor.setpoint = 0;
        }
        Logger.recordOutput("Deployer/PID at goal", pid.atGoal());
        Logger.recordOutput("Deployer/DesiredDeployerVelocity", pid.getSetpoint().velocity);
        Logger.recordOutput("Deployer/DesiredDeployerPosition", pid.getSetpoint().position);
        Logger.recordOutput("Deployer/PID Error", pid.getPositionError());
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
        getData().preset = DeployerPresets.OFF;
    }

}