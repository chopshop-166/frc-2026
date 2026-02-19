package frc.robot.maps;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.wpilibj2.command.Commands.idle;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorControllerGroup;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.DeployerMap;
import frc.robot.maps.subsystems.RollerMap;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.subsystems.Roller;

@RobotMapFor("00:80:2F:40:A6:13")
public class AlphaMap extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap() {
        final double FLOFFSET = 100;
        final double FROFFSET = 179.5;
        final double RLOFFSET = 17.3;
        final double RROFFSET = 47;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(11.379);
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(2);
        final CSSparkMax frontRightSteer = new CSSparkMax(4);
        final CSSparkMax rearLeftSteer = new CSSparkMax(6);
        final CSSparkMax rearRightSteer = new CSSparkMax(8);

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.smartCurrentLimit(30);
        steerConfig.idleMode(IdleMode.kCoast);
        steerConfig.inverted(true);
        frontLeftSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        frontRightSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        rearLeftSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        rearRightSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002),
                new PIDValues(0.05, 0.0, 0.0, 0.21));

        // All Distances are in Meters
        // Front Left Module
        final AnalogEncoder encoderFL = new AnalogEncoder(0, 360, FLOFFSET);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL::get, frontLeftSteer, new CSSparkFlex(1), MK4i_L2);

        // Front Right Module
        final AnalogEncoder encoderFR = new AnalogEncoder(1, 360, FROFFSET);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR::get, frontRightSteer, new CSSparkFlex(3), MK4i_L2);

        // Rear Left Module
        final AnalogEncoder encoderRL = new AnalogEncoder(2, 360, RLOFFSET);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL::get, rearLeftSteer, new CSSparkFlex(5), MK4i_L2);

        // Rear Right Module
        final AnalogEncoder encoderRR = new AnalogEncoder(3, 360, RROFFSET);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR::get, rearRightSteer, new CSSparkFlex(7), MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(15);

        final double maxRotationRadianPerSecond = 2 * Math.PI;

        RobotConfig config = new RobotConfig(58, 4.889, new ModuleConfig(
                0.0508, 6000, 1.0, DCMotor.getNeoVortex(1), 50, 1),
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY));
        PPHolonomicDriveController holonomicDrive = new PPHolonomicDriveController(new PIDConstants(2.0, 0.0, 0.05),
                new PIDConstants(1.0, 0.0, 0.0));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro2, config, holonomicDrive);
    }

    // @Override
    // public ShooterMap getShooterRMap() {
    // CSSparkFlex motorA = new CSSparkFlex(9);
    // CSSparkFlex motorB = new CSSparkFlex(10);
    // SparkFlexConfig configA = new SparkFlexConfig();
    // SparkFlexConfig configB = new SparkFlexConfig();
    // configA.idleMode(IdleMode.kCoast);
    // configA.smartCurrentLimit(60);
    // configA.closedLoop.pid(0, 0, 0);
    // configA.closedLoop.apply(new FeedForwardConfig().kV(0));
    // motorA.setControlType(ControlType.kVelocity);
    // motorB.setControlType(ControlType.kVelocity);
    // motorA.setPidSlot(0);
    // motorB.setPidSlot(0);
    // configA.encoder.quadratureAverageDepth(2)
    // .quadratureMeasurementPeriod(10);

    // ShooterMap.PresetValues presets = preset -> switch (preset) {
    // case CLOSE_SHOT -> RPM.of(3000);
    // case MID_SHOT -> RPM.of(4500);
    // case FAR_SHOT -> RPM.of(6000);
    // case OFF -> RPM.of(0);
    // default -> RPM.of(Double.NaN);
    // };

    // motorA.getMotorController().configure(configA,
    // ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // configB.follow(motorA.getMotorController());

    // motorB.getMotorController().configure(configB,
    // ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // SmartMotorControllerGroup smcg = new SmartMotorControllerGroup(motorA,
    // motorB);

    // return new ShooterMap(smcg, presets);
    // }

    // @Override
    // public ShooterMap getShooterLMap() {
    // CSSparkFlex motorA = new CSSparkFlex(11);
    // CSSparkFlex motorB = new CSSparkFlex(12);
    // SparkFlexConfig configA = new SparkFlexConfig();
    // SparkFlexConfig configB = new SparkFlexConfig();
    // configA.idleMode(IdleMode.kCoast);
    // configA.smartCurrentLimit(60);
    // configA.closedLoop.pid(0, 0, 0);
    // configA.closedLoop.apply(new FeedForwardConfig().kV(0));
    // motorA.setControlType(ControlType.kVelocity);
    // motorB.setControlType(ControlType.kVelocity);
    // motorA.setPidSlot(0);
    // motorB.setPidSlot(0);
    // configA.encoder.quadratureAverageDepth(2)
    // .quadratureMeasurementPeriod(10);

    // ShooterMap.PresetValues presets = preset -> switch (preset) {
    // case CLOSE_SHOT -> RPM.of(3000);
    // case MID_SHOT -> RPM.of(4500);
    // case FAR_SHOT -> RPM.of(6000);
    // case OFF -> RPM.of(0);
    // default -> RPM.of(Double.NaN);
    // };

    // motorA.getMotorController().configure(configA,
    // ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // configB.follow(motorA.getMotorController());

    // motorB.getMotorController().configure(configB,
    // ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // SmartMotorControllerGroup smcg = new SmartMotorControllerGroup(motorA,
    // motorB);

    // return new ShooterMap(smcg, presets);
    // }

    @Override
    public DeployerMap getDeployerMap() {
        CSSparkMax motor = new CSSparkMax(14);
        SparkMaxConfig config = new SparkMaxConfig();
        ProfiledPIDController pid = new ProfiledPIDController(0.25, 0, 0, new Constraints(2 * Math.PI, 6 * Math.PI));
        pid.setTolerance(.1);
        ArmFeedforward feedForward = new ArmFeedforward(0, 0.05, 0.05);
        DutyCycleEncoder encoder = new DutyCycleEncoder(2, 120, 32);
        encoder.setInverted(true);
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
        config.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        config.encoder.positionConversionFactor(((1.0 / 5.0) * (22.0 / 52.0) * (16.0 / 48.0)) * (2
                * Math.PI))
                .velocityConversionFactor((((1.0 / 5.0) * (22.0 / 52.0) * (16.0 / 48.0)) / 60.0) * (2 * Math.PI));
        DeployerMap.PresetValue presets = preset -> switch (preset) {
            case OFF -> Angle.ofBaseUnits(Double.NaN, Degrees);
            case OUT -> Degrees.of(10);
            case IN -> Degrees.of(110);
            default -> Angle.ofBaseUnits(Double.NaN, Degrees);
        };
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return new DeployerMap(motor, encoder, presets, pid, new ValueRange(1, 115), new ValueRange(10, 100),
                feedForward);
    }

    @Override
    public RollerMap getIntakeMap() {
        CSSparkFlex roller = new CSSparkFlex(13);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(60);
        RollerMap.PresetValues presets = preset -> switch (preset) {
            case FORWARD -> .6;
            case REVERSE -> -.5;
            case FORWARD_WIGGLE -> .3;
            case BACKWARDS_WIGGLE -> -.3;
            case OFF -> 0;
            default -> Double.NaN;
        };

        roller.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        return new RollerMap(roller, presets);
    }

    // @Override
    // public RollerMap getFeederMap() {
    // CSSparkFlex roller = new CSSparkFlex(14);
    // SparkFlexConfig config = new SparkFlexConfig();
    // config.idleMode(IdleMode.kCoast);
    // config.smartCurrentLimit(30);
    // RollerMap.PresetValues presets = preset -> switch (preset) {
    // case FORWARD -> .5;
    // case REVERSE -> -.5;
    // case FORWARD_WIGGLE -> .3;
    // case BACKWARDS_WIGGLE -> -.3;
    // case OFF -> 0;
    // default -> Double.NaN;
    // };

    // roller.getMotorController().configure(config, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // return new RollerMap(roller, presets);
    // }

    // @Override
    // public RollerMap getActiveFloorMap() {
    // CSSparkFlex roller = new CSSparkFlex(14);
    // SparkFlexConfig config = new SparkFlexConfig();
    // config.idleMode(IdleMode.kCoast);
    // config.smartCurrentLimit(30);
    // RollerMap.PresetValues presets = preset -> switch (preset) {
    // case FORWARD -> .5;
    // case REVERSE -> -.5;
    // case FORWARD_WIGGLE -> .3;
    // case BACKWARDS_WIGGLE -> -.3;
    // case OFF -> 0;
    // default -> Double.NaN;
    // };

    // roller.getMotorController().configure(config, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // return new RollerMap(roller, presets);
    // }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
