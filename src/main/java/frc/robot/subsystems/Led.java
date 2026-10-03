package frc.robot.subsystems;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.patterns.AlliancePattern;
import com.chopshop166.chopshoplib.leds.patterns.FlashPattern;
import com.chopshop166.chopshoplib.leds.patterns.RainbowRoad;
import com.chopshop166.chopshoplib.leds.patterns.SpinPattern;
import com.chopshop166.chopshoplib.maps.LedMapBase;
import com.chopshop166.chopshoplib.maps.WPILedMap;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class Led extends LEDSubsystem {

    public Led(LedMapBase map) {
        super(map);
        // This one is length / 2 because the buffer has a mirrored other half

    }

    public Command colorAlliance() {
        return setPattern("Alliance", new AlliancePattern(), "Alliance");
    }

    public Command coloralliance() {
        return setGlobalPattern(new AlliancePattern());
    }

    public Command resetColor() {
        return setGlobalPattern(new Color(201, 198, 204));
    }

    public Command spinRed() {
        return setPattern("Shooter", new SpinPattern(new Color(144, 0, 0)), "Spinning");
    }

    public Command spinGreen() {
        return setPattern("Shooter", new SpinPattern(new Color(0, 144, 0)), "Spinning");
    }

    public Command flashRed() {
        return setPattern("underglow", new FlashPattern(new Color(255, 0, 0), .2), "AWESOME");
    }

    public Command flashGreen() {
        return setPattern("underglow", new FlashPattern(new Color(0, 144, 0), .2), "AWESOME");
    }

    public Command flash() {
        return setGlobalPattern(new FlashPattern(new Color(255, 32, 82), .5));
    }

    public Command blue() {
        return setGlobalPattern(new Color(255, 0, 0));
    }

    public Command rainbow() {
        return setPattern("underglow", new RainbowRoad(), "underglow");
    }

}