// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.ctre.phoenix.schedulers.SequentialScheduler;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import frc.robot.maps.subsystems.HoodMap.HoodPresets;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;
import frc.robot.subsystems.Deployer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationTargets;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

public final class Robot extends CommandRobot {

    private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    // Helpers
    final DoubleUnaryOperator driveScaler = getScaler(0.45, 0.25);
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    // Subsystems
    // Make them public so that the sequences can access them
    public Drive drive = new Drive(map.getDriveMap(), () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftX());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftY());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getRightX());
    }, map.getVisionMap());

    public Shooter shooter = new Shooter(map.getShooterMap(), "Shooter");
    public Roller intake = new Roller(map.getIntakeMap(), "Intake");
    public Deployer deployer = new Deployer(map.getDeployerMap(),
            RobotUtils.deadbandAxis(.1, () -> copilotController.getLeftY()));
    public Roller feeder = new Roller(map.getFeederMap(), "Feeder");
    public Roller activeFloor = new Roller(map.getActiveFloorMap(), "ActiveFloor");
    public Hood hood = new Hood(map.getHoodMap(), RobotUtils.deadbandAxis(.1, () -> copilotController.getRightY()));

    // Things that use all the subsystems
    private CommandSequences sequences = new CommandSequences(this);

    // The default auto
    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    private final SendableChooser<Command> autoChooser;

    public Robot() {
        super();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    @Override
    public void robotInit() {
        super.robotInit();

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes committed";
            case 1 -> "Uncomitted changes";
            default -> "Unknown";
        });
        Logger.recordMetadata("RobotMap", map.getClass().getName());

        map.setupLogging();

        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values
        // may be added.
        Logger.start();

        CommandScheduler.getInstance().schedule(sequences.operatorSafeState());
        DriverStation.silenceJoystickConnectionWarning(true);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            Logger.recordOutput("Drive/PathPlannerTargetPose", pose);
        });

        CommandScheduler.getInstance().onCommandInterrupt((oldCmd, newCmd) -> {
            if (!DriverStation.isFMSAttached()) {
                String newSub = "<NONE>";
                String newName = "<NONE>";
                if (newCmd.isPresent()) {
                    newSub = newCmd.get().getSubsystem();
                    newName = newCmd.get().getName();
                }
                System.out.println("Command interrupt: `" + oldCmd.getSubsystem() + "/" +
                        oldCmd.getName() + "` -> `" + newSub + "/" + newName + "`");
            }
        });
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        CommandScheduler.getInstance().schedule(sequences.operatorSafeState().ignoringDisable(true));
    }

    @Override
    public void configureButtonBindings() {
        driveController.leftBumper().whileTrue(drive.rotateToTargetContinuous(RotationTargets.HUB))
                .onFalse(drive.rotationTargetOff());
        // copilot stop
        copilotController.start().onTrue(sequences.operatorSafeState());
        copilotController.back().onTrue(hood.zero().ignoringDisable(true));
        // feed shooter

        // // Intake
        copilotController.a().whileTrue(sequences.intake())
                .onFalse(intake.safeStateCmd());
        // copilotController.b().whileTrue(sequences.shootAutoAlign(ShooterPresets.CLOSE_SHOT,
        // HoodPresets.CLOSE));
        // copilotController.b().whileTrue(sequences.shootAutoAlign(ShooterPresets.AUTO_SPEED,
        // HoodPresets.AUTO_ANGLE));
        // copilotController.b()
        // .whileTrue(
        // hood.moveToAngle(HoodPresets.AUTO_ANGLE).alongWith(drive.rotateToTarget(RotationTargets.HUB)))
        // .onFalse(hood.safeStateCmd());
        // .onFalse(sequences.operatorSafeState());

        copilotController.b().whileTrue(sequences.shootAutoAlign(ShooterPresets.AUTO_SPEED, HoodPresets.AUTO_ANGLE))
                .onFalse(sequences.operatorSafeState().andThen(hood.moveToAngle(HoodPresets.DOWN)));
        copilotController.x().whileTrue(sequences.shoot(ShooterPresets.CLOSE_SHOT, HoodPresets.MID))
                .onFalse(sequences.operatorSafeState().andThen(hood.moveToAngle(HoodPresets.DOWN)));
        // copilotController.x().whileTrue(sequences.shoot(ShooterPresets.CLOSE_SHOT,
        // HoodPresets.CLOSE))
        // .onFalse(sequences.operatorSafeState());
        copilotController.y().whileTrue(sequences.shoot(ShooterPresets.FAR_SHOT, HoodPresets.FAR))
                .onFalse(sequences.operatorSafeState().andThen(hood.moveToAngle(HoodPresets.DOWN)));
        copilotController.rightBumper().onTrue(sequences.retractIntake());
        copilotController.leftBumper().onTrue(sequences.rollOut()).onFalse(sequences.operatorSafeState());
        // copilotController.y().whileTrue(sequences.shoot(ShooterPresets.NETWORK_TABLES,
        // HoodPresets.NETWORK_TABLES));

    }

    private final void registerNamedCommands() {
        NamedCommands.registerCommand("Intake", sequences.intake().withName("Intake!!!"));
        NamedCommands.registerCommand("Shoot",
                sequences.shootAutoAlign(ShooterPresets.AUTO_SPEED, HoodPresets.AUTO_ANGLE).withName("Shoot"));
        NamedCommands.registerCommand("Stop Shooting",
                sequences.operatorSafeState().andThen(hood.moveToAngle(HoodPresets.DOWN)).withName("Stop shooting"));
    }

    @Override
    public void populateDashboard() {
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    @Override

    public void setDefaultCommands() {
        // hood.setDefaultCommand(hood.manualControl(RobotUtils.deadbandAxis(.1, () ->
        // -copilotController.getRightY())));
    }

    public DoubleUnaryOperator getScaler(double leftRange, double rightRange) {
        return speed -> {
            double leftTrigger = driveController.getLeftTriggerAxis();
            double rightTrigger = driveController.getRightTriggerAxis();
            double modifier = (rightRange * rightTrigger) - (leftRange * leftTrigger) + 0.75;
            return modifier * speed;
        };
    }

}
