package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {
    public SmartMotorController roller;

    public ShooterMap() {
        this(new SmartMotorController());
    }

    public ShooterMap(SmartMotorController roller) {
        this.roller = roller;
    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(roller);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData();

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
    }
}
