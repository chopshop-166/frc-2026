package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class IntakeMap implements LoggableMap<IntakeMap.Data> {

    public SmartMotorController roller;

    public IntakeMap() {
        this(new SmartMotorController());

    }

    public IntakeMap(SmartMotorController roller) {
        this.roller = roller;
    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(roller);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData();
    }
}
