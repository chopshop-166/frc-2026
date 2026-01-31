package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.FeederMap;
import frc.robot.maps.subsystems.FeederMap.Data;

public class Feeder extends LoggedSubsystem<Data, FeederMap> {

    private final double FEED_SPEED = .5;
    private final double REVERSE_SPEED = -.5;

    public Feeder(FeederMap feederMap) {
        super(new Data(), feederMap);
    }

    public Command feed() {
        return runOnce(() -> {
            getData().feeder.setpoint = FEED_SPEED;
        });
    }

    public Command reverse() {
        return runOnce(() -> {
            getData().feeder.setpoint = REVERSE_SPEED;
        });
    }

    @Override
    public void safeState() {
        getData().feeder.setpoint = 0;
    }

}
