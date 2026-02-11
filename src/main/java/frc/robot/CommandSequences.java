package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;
import frc.robot.subsystems.Drive.RotationTargets;

public class CommandSequences {

    Robot robot;

    public CommandSequences(Robot robot) {
        this.robot = robot;
    }

    // make sequences for intake and shooter.

    public Command Intake() {
        return runOnce(() -> {
            robot.deployer.moveTo(DeployerPresets.OUT).alongWith(robot.intake.rollIn());
        });
    }

    public Command Shoot(ShooterPresets shotSpeed) {
        return runOnce(() -> {
            robot.shooterL.shoot(shotSpeed)
                    .alongWith(robot.shooterR.shoot(shotSpeed), robot.drive.rotateToTarget(RotationTargets.HUB))
                    .andThen(robot.activeFloor.rollIn(), robot.feeder.rollIn());
        });
    }

    public Command OperatorSafeState() {
        return runOnce(() -> {
            robot.intake.safeStateCmd().alongWith(robot.deployer.safeStateCmd(), robot.activeFloor.safeStateCmd(),
                    robot.feeder.safeStateCmd(), robot.shooterL.safeStateCmd(), robot.shooterR.safeStateCmd());
        });
    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }
}
