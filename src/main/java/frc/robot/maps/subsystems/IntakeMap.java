package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

public class IntakeMap implements LoggableMap<IntakeMap.Data> {
    public SmartMotorController roller;
    public SmartMotorController kicker;

    public IntakeMap() {
        this(new SmartMotorController(), new SmartMotorController()) ;
        
    }

    public IntakeMap(SmartMotorController roller, SmartMotorController kicker) {
        this.roller = roller;
        this.kicker = kicker;
    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(roller);
        data.roller.updateData(kicker);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData();
        public MotorControllerData kicker = new MotorControllerData();

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
    }
}
