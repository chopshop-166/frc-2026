package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;

public class FeederMap implements LoggableMap<FeederMap.Data> {
    public SmartMotorController feeder;

    public FeederMap() {
        this(new SmartMotorController());
    }

    public FeederMap(SmartMotorController feeder) {
        this.feeder = feeder;
    }

    @Override
    public void updateData(Data data) {
        data.feeder.updateData(feeder);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData feeder = new MotorControllerData();
    }
}
