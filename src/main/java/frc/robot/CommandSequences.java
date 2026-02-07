package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeployerMap.DeployerPresets;
import frc.robot.maps.subsystems.ShooterMap.ShooterPresets;

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
            // TODO: Add PID to hub when vision is merged
            robot.shooterL.shoot(shotSpeed).alongWith(robot.shooterR.shoot(shotSpeed))
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
