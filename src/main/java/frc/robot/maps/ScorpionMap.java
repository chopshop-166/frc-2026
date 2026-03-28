package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.CameraSource;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.VisionMap;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorControllerGroup;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.maps.subsystems.DeployerMap;
import frc.robot.maps.subsystems.HoodMap;
import frc.robot.maps.subsystems.HoodMap.PresetValue;
import frc.robot.maps.subsystems.RollerMap;
import frc.robot.maps.subsystems.ShooterMap;

@RobotMapFor("00:80:2F:40:A6:13")
public class ScorpionMap extends RobotMap {

    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    DoubleSubscriber distanceToHubSub = ntInstance.getDoubleTopic("Drive/Distance To Hub").subscribe(0);

    private final double CLOSE_SHOT_RPM = 1800;
    private final double MID_SHOT_RPM = 2000;
    private final double FAR_SHOT_RPM = 2500;

    @Override
    public SwerveDriveMap getDriveMap() {
        final double FLOFFSET = 100 + 180;
        final double FROFFSET = 110 + 180;
        final double RLOFFSET = 16 + 180;
        final double RROFFSET = 45 + 180;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(11.379);
        final Pigeon2 pigeon = new Pigeon2(1);
        var pigeonConfig = pigeon.getConfigurator();
        pigeonConfig.apply(new MountPoseConfigs().withMountPoseRoll(180));
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(pigeon);

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
                maxRotationRadianPerSecond, pigeonGyro2, config, holonomicDrive,
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01));
    }

    @Override
    public ShooterMap getShooterMap() {
        CSSparkFlex motorA = new CSSparkFlex(11);
        CSSparkFlex motorB = new CSSparkFlex(12);
        // Motors C & D are on the right if shooter is facing you
        CSSparkFlex motorC = new CSSparkFlex(9);
        CSSparkFlex motorD = new CSSparkFlex(10);
        SparkFlexConfig configA = new SparkFlexConfig();
        SparkFlexConfig configB = new SparkFlexConfig();
        SparkFlexConfig configCD = new SparkFlexConfig();
        SparkFlexConfig configD = new SparkFlexConfig();
        configA.idleMode(IdleMode.kCoast);

        configA.smartCurrentLimit(60);
        configB.smartCurrentLimit(60);
        configCD.smartCurrentLimit(60);

        configA.closedLoop.pid(0, 0, 0);
        configA.closedLoop.apply(new FeedForwardConfig().kV(0));

        motorA.setControlType(ControlType.kVelocity);
        motorB.setControlType(ControlType.kVelocity);
        motorC.setControlType(ControlType.kVelocity);
        motorD.setControlType(ControlType.kVelocity);

        motorA.setPidSlot(0);
        motorB.setPidSlot(0);
        motorC.setPidSlot(0);
        motorD.setPidSlot(0);

        configA.encoder.quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        configB.encoder.quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        configCD.encoder.quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);

        SmartDashboard.putNumber("Shooter", 2000);
        ShooterMap.PresetValues presets = preset -> switch (preset) {
            case CLOSE_SHOT -> CLOSE_SHOT_RPM;
            case MID_SHOT -> MID_SHOT_RPM;
            case FAR_SHOT -> FAR_SHOT_RPM;
            case OFF -> 0;
            case NETWORK_TABLES -> SmartDashboard.getNumber("Shooter", 2000);
            case AUTO_SPEED -> Math.min(2500, (distanceToHubSub.getAsDouble() + 8.3804) / 0.0059);
            default -> Double.NaN;
        };

        motorA.getMotorController().configure(configA,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        configB.follow(motorA.getMotorController());
        configCD.follow(motorA.getMotorController(), true);

        motorB.getMotorController().configure(configB,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        motorC.getMotorController().configure(configCD,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        motorD.getMotorController().configure(configCD,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        SmartMotorControllerGroup smcg = new SmartMotorControllerGroup(motorA, motorB, motorC, motorD);

        return new ShooterMap(smcg, presets);
    }

    @Override
    public DeployerMap getDeployerMap() {
        CSSparkMax motor = new CSSparkMax(14);
        SparkMaxConfig config = new SparkMaxConfig();
        ProfiledPIDController pid = new ProfiledPIDController(0.27, 0, 0, new Constraints(2 * Math.PI, 6 * Math.PI));
        pid.setTolerance(.1);
        ArmFeedforward feedForward = new ArmFeedforward(0, 0.05, 0.08);
        DutyCycleEncoder encoder = new DutyCycleEncoder(2, 120, 35);
        encoder.setInverted(true);
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
        config.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        config.encoder.positionConversionFactor(((1.0 / 9.0) * (22.0 / 52.0) * (18.0 / 48.0)) * (2
                * Math.PI))
                .velocityConversionFactor((((1.0 / 5.0) * (22.0 / 52.0) * (16.0 / 48.0)) / 60.0) * (2 * Math.PI));
        DeployerMap.PresetValue presets = preset -> switch (preset) {
            case OFF -> Double.NaN;
            case OUT -> Units.degreesToRadians(3);
            case IN -> Units.degreesToRadians(118);
            case WIGGLE_IN -> Units.degreesToRadians(84);
            default -> Double.NaN;
        };
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return new DeployerMap(motor, encoder, presets, pid,
                new ValueRange(Units.degreesToRadians(1), Units.degreesToRadians(119)),
                new ValueRange(Units.degreesToRadians(10), Units.degreesToRadians(100)),
                feedForward);
    }

    @Override
    public RollerMap getIntakeMap() {
        // Right motor is on the right when intake is facing you
        CSSparkFlex motorRight = new CSSparkFlex(13);
        CSSparkFlex motorLeft = new CSSparkFlex(19);
        SparkFlexConfig configRight = new SparkFlexConfig();
        SparkFlexConfig configLeft = new SparkFlexConfig();

        configRight.idleMode(IdleMode.kCoast);
        configLeft.idleMode(IdleMode.kCoast);

        configRight.smartCurrentLimit(40);
        configLeft.smartCurrentLimit(40);

        configLeft.follow(motorRight.getMotorController(), true);
        RollerMap.PresetValues presets = preset -> switch (preset) {
            case FORWARD -> 0.8;
            case REVERSE -> -0.8;
            case FORWARD_WIGGLE -> .3;
            case BACKWARDS_WIGGLE -> -.3;
            case OFF -> 0;
            default -> Double.NaN;
        };

        motorRight.getMotorController().configure(configRight, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        motorLeft.getMotorController().configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SmartMotorControllerGroup smcg = new SmartMotorControllerGroup(motorRight, motorLeft);
        return new RollerMap(smcg, presets);
    }

    @Override
    public RollerMap getFeederMap() {
        CSSparkMax roller = new CSSparkMax(16);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast).inverted(true);
        config.smartCurrentLimit(50);
        RollerMap.PresetValues presets = preset -> switch (preset) {
            case FORWARD -> 1.0;
            case REVERSE -> -0.5;
            case FORWARD_WIGGLE -> .3;
            case BACKWARDS_WIGGLE -> -.3;
            case OFF -> 0;
            default -> Double.NaN;
        };

        roller.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        return new RollerMap(roller, presets);
    }

    @Override
    public RollerMap getActiveFloorMap() {
        CSSparkMax rollerR = new CSSparkMax(15);
        CSSparkMax rollerL = new CSSparkMax(18);
        SparkMaxConfig configR = new SparkMaxConfig();
        SparkMaxConfig configL = new SparkMaxConfig();
        configR.idleMode(IdleMode.kCoast).inverted(true);
        configR.smartCurrentLimit(30);
        RollerMap.PresetValues presets = preset -> switch (preset) {
            case FORWARD -> 1;
            case REVERSE -> -0.5;
            case FORWARD_WIGGLE -> .3;
            case BACKWARDS_WIGGLE -> -.3;
            case OFF -> 0;
            default -> Double.NaN;
        };

        rollerR.getMotorController().configure(configR, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        configL.follow(rollerR.getMotorController(), true);
        rollerL.getMotorController().configure(configL, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SmartMotorControllerGroup smcg = new SmartMotorControllerGroup(rollerR, rollerL);

        return new RollerMap(smcg, presets);
    }

    @Override
    public HoodMap getHoodMap() {
        CSSparkFlex motor = new CSSparkFlex(17);
        ProfiledPIDController pid = new ProfiledPIDController(.43, 0, 0,
                new Constraints(Math.PI, Math.PI));
        pid.setTolerance(Units.degreesToRadians(.5));
        SparkFlexConfig config = new SparkFlexConfig();
        ArmFeedforward feedForward = new ArmFeedforward(0, 0.02, 0.07);
        double gearRatio = (1.0 / 9.0) * (15.0 / 965.0) * (2.0 * Math.PI);
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        config.encoder.positionConversionFactor(gearRatio)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10)
                .velocityConversionFactor(gearRatio / 60.0);
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        SmartDashboard.putNumber("Hood/angle", 0);
        PresetValue presets = preset -> switch (preset) {
            case CLOSE -> 0.15;
            case MID -> 0.2;
            case FAR -> 0.44;
            case OFF -> Double.NaN;
            case NETWORK_TABLES -> SmartDashboard.getNumber("Hood/angle", 0);
            default -> Double.NaN;
        };

        return new HoodMap(motor, pid, new ValueRange(0, .48), feedForward, presets);
    }

    @Override
    public VisionMap getVisionMap() {

        return new VisionMap(0,
                new CameraSource("L_Scorpion_Cam",
                        new Transform3d(Units.inchesToMeters(4.227), Units.inchesToMeters(10.971),
                                Units.inchesToMeters(21.149),
                                new Rotation3d(0, Units.degreesToRadians(-27), Units.degreesToRadians(0)))),
                new CameraSource("R_Scorpion_Cam",
                        new Transform3d(Units.inchesToMeters(4.227), Units.inchesToMeters(-10.954),
                                Units.inchesToMeters(21.223),
                                new Rotation3d(0, Units.degreesToRadians(-27), Units.degreesToRadians(0)))));
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
