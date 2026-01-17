package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {
    private final double GRAB_SPEED = 0.5;
    private final double RELEASE_SPEED = -0.5;

    public Shooter(ShooterMap shooterMap) {
        super(new Data(), shooterMap);
    }

    public Command spinIn() {
        return runSafe(() -> {
            getData().roller.setpoint = GRAB_SPEED;
        });
    }

    public Command spinOut() {
        return runSafe(() -> {
            getData().roller.setpoint = RELEASE_SPEED;
        });
    }

    @Override
    public void reset() {
        // nothing to reset
    }

    @Override
    public void safeState() {
        getData().roller.setpoint = 0;
    }

}
