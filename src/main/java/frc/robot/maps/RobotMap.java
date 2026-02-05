package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.LedMapBase;
import com.chopshop166.chopshoplib.maps.MockLedMap;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.VisionMap;

import frc.robot.maps.subsystems.DeployerMap;
import frc.robot.maps.subsystems.HoodMap;
import frc.robot.maps.subsystems.KickerMap;
import frc.robot.maps.subsystems.RollerMap;
import frc.robot.maps.subsystems.ShooterMap;

public class RobotMap {

    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public VisionMap getVisionMap() {
        return new VisionMap();
    }

    public LedMapBase getLedMap() {
        return new MockLedMap();
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();
    }

    public KickerMap getKickerMap() {
        return new KickerMap();
    }

    public RollerMap getIntakeMap() {
        return new RollerMap();
    }

    public DeployerMap getDeploymentMap() {
        return new DeployerMap();
    }

    public RollerMap getFeederMap() {
        return new RollerMap();
    }

    public RollerMap getActiveFloorMap() {
        return new RollerMap();
    }

    public HoodMap getHoodMap() {
        return new HoodMap();
    }

    public void setupLogging() {
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
}
