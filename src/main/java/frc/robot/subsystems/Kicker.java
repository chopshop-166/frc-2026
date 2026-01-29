package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.KickerMap;
import frc.robot.maps.subsystems.KickerMap.Data;

public class Kicker  extends LoggedSubsystem<Data, KickerMap>{
    private final double KICK_OUT_SPEED = -0.5;
    private final double KICK_IN_SPEED = 0.5;

    public Kicker(KickerMap kickerMap) {
        super(new Data(), kickerMap);
    }

    public Command kickIn() {
        return runSafe(() -> {
            getData().kicker.setpoint = KICK_IN_SPEED;
        });
    }

    public Command kickOut() {
        return runSafe(() -> {
            getData().kicker.setpoint = KICK_OUT_SPEED;
        });
    }


    @Override
    public void reset() {
        // nothing to reset
    }

    @Override
    public void safeState() {
        getData().kicker.setpoint = 0;
    }

}
