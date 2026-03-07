package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    private final double TOLERANCE = 100;

    private final String implName;
    DoubleSupplier shooterSpeed;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoublePublisher shooterVelocityFPSPub = instance.getDoubleTopic(getName() + "/Linear Velocity").publish();

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    // Create a new SysId routine for characterizing the shooter.
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    voltage -> getMap().flywheel.setVoltage(voltage),
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("shooter-wheel")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                getMap().flywheel.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(
                                        m_angle.mut_replace(getMap().flywheel.getEncoder().getDistance(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(
                                                getMap().flywheel.getEncoder().getRate(), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in WPILog with this subsystem's name ("shooter")
                    this));

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Shooter(ShooterMap shooterMap, String name) {
        super(new Data(), shooterMap);
        this.implName = name;
    }

    @Override
    public String getName() {
        return implName;
    }

    public Command spinUp(ShooterPresets presetSpeed) {
        Debouncer debouncer = new Debouncer(.2, DebounceType.kBoth);

        return runOnce(() -> {
            setPreset(presetSpeed);
        }).andThen(Commands.waitUntil(() -> debouncer.calculate(Math.abs(getError()) < TOLERANCE)));

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
        Logger.recordOutput(getName() + "/PID Error", getError());

    }

    @Override
    public void safeState() {
        setPreset(ShooterPresets.OFF);
    }

    private double getError() {
        return getData().flywheel.setpoint - getData().flywheel.velocity;
    }

}
