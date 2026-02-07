package com.chopshop166.chopshoplib.maps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class VisionMap {

    /** Camera-based position estimators. */
    public final List<CameraSource> visionSources;
    public double offset;

    /** Default constructor. */
    public VisionMap() {
        this(0.0, new ArrayList<>());
    }

    /**
     * Constructor.
     * 
     * @param visionSources Any number of vision sources.
     */
    public VisionMap(double offset, final CameraSource... visionSources) {
        this(offset, Arrays.asList(visionSources));
    }

    /**
     * Constructor.
     * 
     * @param visionSources A list of vision sources.
     */
    public VisionMap(double offset, final List<CameraSource> visionSources) {
        this.visionSources = visionSources;
        this.offset = offset;
    }

    /**
     * Update the data in a pose estimator with the poses from all cameras.
     * 
     * @param <T>       Estimator wheel type.
     * @param estimator The WPIlib estimator object.
     */
    public <T> void updateData(Data data) {
        for (var source : this.visionSources) {
            var results = source.camera.getAllUnreadResults();
            Logger.recordOutput("Sees Tags", !results.isEmpty());
            if (!results.isEmpty()) {
                for (PhotonPipelineResult result : results) {
                    if ((result.multitagResult.isPresent()
                            && result.multitagResult.get().estimatedPose.ambiguity > 0.50)) {
                        continue;
                    }
                    // Put the results measurement into the pose estimator
                    var estimate = source.estimator.estimateCoprocMultiTagPose(result);
                    estimate.ifPresent(est -> {
                        Logger.recordOutput("Camera Pose Estimate/" + source.camera.getName(),
                                est.estimatedPose.toPose2d());
                        data.estimator.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                est.timestampSeconds);
                    });
                }
            }
        }
    }

    public static class Data {
        public SwerveDrivePoseEstimator estimator;
    }
}
