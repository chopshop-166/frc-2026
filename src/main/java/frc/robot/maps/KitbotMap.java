package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

@RobotMapFor("Kitbot")
public class KitbotMap extends RobotMap {
    @Override
    public SwerveDriveMap getDriveMap() {

        // Use Phoenix tuner
        // CAN ID 2
        final double FLOFFSET = -0.972;
        // CAN ID 4
        final double FROFFSET = -0.261;
        // CAN ID 1
        final double RLOFFSET = -0.85;
        // CAN ID 3
        final double RROFFSET = -0.9638;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(10.875);
        final PigeonGyro2 pigeonGyro = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        CSSparkFlex frontLeftDrive = new CSSparkFlex(3);
        CSSparkFlex frontRightDrive = new CSSparkFlex(7);
        CSSparkFlex rearLeftDrive = new CSSparkFlex(1);
        CSSparkFlex rearRightDrive = new CSSparkFlex(5);

        frontLeftSteer.setInverted(true);
        frontRightSteer.setInverted(true);
        rearLeftSteer.setInverted(true);
        rearRightSteer.setInverted(true);

        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.MountPoseRoll = 180;
        pigeonGyro.getRaw().getConfigurator().apply(pigeonConfig, MODULE_OFFSET_XY);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002),
                new PIDValues(0.05, 0.0, 0.0, 0.21));

        // All Distances are in Meters
        // Front left module
        final CANcoder encoderFL = new CANcoder(2);
        CANcoderConfiguration encoderFLConfig = new CANcoderConfiguration();
        encoderFLConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderFLConfig.MagnetSensor.MagnetOffset = FLOFFSET;
        encoderFL.getConfigurator().apply(encoderFLConfig);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderFL), frontLeftSteer, frontLeftDrive, MK4i_L2);
        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderFRConfig.MagnetSensor.MagnetOffset = FROFFSET;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, frontRightDrive, MK4i_L2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(1);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderRLConfig.MagnetSensor.MagnetOffset = RLOFFSET;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, rearLeftDrive, MK4i_L2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(3);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderRRConfig.MagnetSensor.MagnetOffset = RROFFSET;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, rearRightDrive, MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(3);

        final double maxRotationRadianPerSecond = 2 * Math.PI;

        RobotConfig config = new RobotConfig(68, 5000, new ModuleConfig(
                0.1016, 6000, 1.0, DCMotor.getNEO(1), 50, 1),
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY));
        PPHolonomicDriveController holonomicDrive = new PPHolonomicDriveController(new PIDConstants(2.0, 0.0, 0.05),
                new PIDConstants(1.0, 0.0, 0.0));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro,
                config, holonomicDrive);
    }
}
