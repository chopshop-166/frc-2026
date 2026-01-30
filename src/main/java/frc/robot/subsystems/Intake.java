package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.IntakeMap.Data;

public class Intake extends LoggedSubsystem<Data, IntakeMap> {

    private final double INTAKE_SPEED = 0.5;
    private final double EXPEL_SPEED = -0.5;

    public Intake(IntakeMap intakeMap) {
        super(new Data(), intakeMap);
    }

    public Command intake() {
        return runOnce(() -> {
            getData().roller.setpoint = INTAKE_SPEED;
        });
    }

    public Command expel() {
        return runOnce(() -> {
            getData().roller.setpoint = EXPEL_SPEED;
        });
    }

    @Override
    public void safeState() {
        getData().roller.setpoint = 0;
    }

}
