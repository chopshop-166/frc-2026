package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class KickerMap implements LoggableMap<KickerMap.Data> {
    public SmartMotorController kicker;

    public KickerMap() {
        this(new SmartMotorController());

    }

    public KickerMap(SmartMotorController kicker) {
        this.kicker = kicker;
    }

    @Override
    public void updateData(Data data) {
        data.kicker.updateData(kicker);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData kicker = new MotorControllerData();

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
    }
}
