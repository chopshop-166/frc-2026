package frc.robot;

import java.util.Collections;
import java.util.Comparator;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.maps.CameraSource;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

    public static Pose2d getHubCenter(boolean isBlueAlliance) {
        Pose2d translation;
        if (!isBlueAlliance) {
            Pose3d poseLeft = kTagLayout.getTagPose(10).get();
            Pose3d poseRight = kTagLayout.getTagPose(4).get();
            Logger.recordOutput("Reef center", "Blue");
            Translation2d translationLeft = poseLeft.getTranslation().toTranslation2d();
            Translation2d translationRight = poseRight.getTranslation().toTranslation2d();
            translation = new Pose2d(translationLeft.plus(translationRight).div(2), Rotation2d.kZero);
        } else {
            Pose3d poseLeft = kTagLayout.getTagPose(20).get();
            Pose3d poseRight = kTagLayout.getTagPose(26).get();
            Logger.recordOutput("Reef center", "Red");
            Translation2d translationLeft = poseLeft.getTranslation().toTranslation2d();
            Translation2d translationRight = poseRight.getTranslation().toTranslation2d();
            translation = new Pose2d(translationLeft.plus(translationRight).div(2), Rotation2d.kZero);
        }

        return translation;
    }
}
