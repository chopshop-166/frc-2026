package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.patterns.AlliancePattern;
import com.chopshop166.chopshoplib.leds.patterns.FirePattern;
import com.chopshop166.chopshoplib.leds.patterns.FlashPattern;
import com.chopshop166.chopshoplib.leds.patterns.RainbowRoad;
import com.chopshop166.chopshoplib.leds.patterns.SpinPattern;
import com.chopshop166.chopshoplib.maps.LedMapBase;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class Led extends LEDSubsystem {

    Color shooterReadyColor = new Color(0, 255, 0); // green
    Color shooterPrepColor = new Color(255, 0, 0); // red
    Color visionAlignedColor = new Color(0, 0, 255); // blue
    Color intakeColor = new Color(255, 255, 0);

    public Led(LedMapBase map) {
        super(map);
    }

    public Command colorAlliance() {
        return setGlobalPattern(new AlliancePattern());
    }

    public Command resetColor() {
        return setGlobalPattern(new Color(201, 198, 204));
    }

    public Command intaking() {
        Logger.recordOutput("LEDS", "Intake Flashing");
        return setPattern("Intake", new FlashPattern(intakeColor, .15), "Flashing");
    }

    public Command outtaking() {
        Logger.recordOutput("LEDS", "Intake Spining");
        return setPattern("Intake", new SpinPattern(intakeColor), "Spining");
    }

    public Command ShooterPrep() {
        Logger.recordOutput("LEDS", "Shooter Prep");
        return setPattern("Shooter", new SpinPattern(shooterPrepColor));
    }

    public Command ShooterReady() {
        Logger.recordOutput("LEDS", "Shooter Ready");
        return setPattern("Shooter", new FlashPattern(shooterReadyColor, .15));
    }

    public Command visionAligning() {
        Logger.recordOutput("LEDS", "Rainbow");
        return setPattern("Vision", new RainbowRoad(), "Aligning");
    }

    public Command visionAligned() {
        Logger.recordOutput("LEDS", "Flashing");
        return setPattern("Vision", new FlashPattern(visionAlignedColor, 0.125), "Aligned");
    }
}
