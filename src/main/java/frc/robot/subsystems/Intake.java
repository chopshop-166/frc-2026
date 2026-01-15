package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.IntakeMap.Data;

public class Intake extends LoggedSubsystem<Data, IntakeMap> {
    private final double GRAB_SPEED = 0.5;
    private final double RELEASE_SPEED = -0.5;
    private final double KICK_SPEED = 0.5;

    public Intake(IntakeMap intakemap) {
        super(new Data(), intakemap);
    }

    public Command KickIn() {
        return runSafe(() -> {
            getData().kicker.setpoint = KICK_SPEED;
        });
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
        getData().kicker.setpoint = 0;
    }

}
