package frc.robot;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.maps.CameraSource;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision {

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = CameraSource.DEFAULT_FIELD;

    public Vision() {
    }

    public static int getNearestTagIdImpl(Map<Integer, Pose2d> poses, SwerveDrivePoseEstimator estimator) {
        Pose2d robotPose = estimator.getEstimatedPosition();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();
        return Collections.min(
                poses.entrySet(),
                Comparator.comparing((Map.Entry<Integer, Pose2d> entry) -> {
                    return robotTranslation.getDistance(entry.getValue().getTranslation());
                }).thenComparing((Map.Entry<Integer, Pose2d> entry) -> {
                    return Math.abs(robotRotation.minus(entry.getValue().getRotation()).getRadians());
                })).getKey();
    }

    public static Translation2d getHubCenter(boolean isBlueAlliance) {
        Translation3d translation;
        if (isBlueAlliance) {
            Translation3d poseLeft = kTagLayout.getTagPose(10).get().getTranslation();
            Translation3d poseRigth = kTagLayout.getTagPose(4).get().getTranslation();
            Logger.recordOutput("Reef center", "Blue");
            translation = poseLeft.plus(poseRigth).div(2);
        } else {
            Translation3d poseLeft = kTagLayout.getTagPose(20).get().getTranslation();
            Translation3d poseRight = kTagLayout.getTagPose(26).get().getTranslation();
            Logger.recordOutput("Reef center", "Red");
            translation = poseLeft.plus(poseRight).div(2);
        }

        return translation.toTranslation2d();
    }
}
