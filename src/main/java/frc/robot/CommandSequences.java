package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import frc.robot.maps.subsystems.HoodMap.HoodPresets;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationTargets;

public class CommandSequences {

    Robot robot;

    public CommandSequences(Robot robot) {
        this.robot = robot;
    }

    // make sequences for intake and shooter.

    public Command intake() {
        return robot.deployer.moveTo(DeployerPresets.OUT).alongWith(robot.intake.rollIn());
    }

    public Command retractIntake() {
        return robot.deployer.moveTo(DeployerPresets.IN).alongWith(robot.intake.safeStateCmd());
    }

    public Command shootAutoAlign(ShooterPresets shotSpeed, HoodPresets hoodAngle) {
        return robot.hood.moveToAngle(hoodAngle)
                .alongWith(robot.shooterL.spinUp(shotSpeed).alongWith(robot.shooterR.spinUp(shotSpeed),
                        robot.drive.rotateToTarget(Drive.RotationTargets.HUB))
                        .andThen(feedShooter()
                                .alongWith(robot.drive.rotateToTargetContinuous(Drive.RotationTargets.HUB))));
    }

    public Command shoot(ShooterPresets shotSpeed, HoodPresets hoodangle) {
        return robot.hood.moveToAngle(hoodangle)
                .alongWith(robot.shooterL.spinUp(shotSpeed).alongWith(robot.shooterR.spinUp(shotSpeed)
                        .andThen(feedShooter())));
    }

    public Command feedShooter() {
        return robot.feeder.rollIn().alongWith(robot.activeFloor.rollIn(), robot.intake.rollIn());
    }

    public Command operatorSafeState() {
        return robot.intake.safeStateCmd().alongWith(robot.deployer.safeStateCmd(), robot.activeFloor.safeStateCmd(),
                robot.feeder.safeStateCmd(), robot.shooterL.safeStateCmd(), robot.shooterR.safeStateCmd(),
                robot.drive.rotationTargetOff());

    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }
}
