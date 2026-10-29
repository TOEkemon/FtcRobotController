package org.firstinspires.ftc.teamcode.utilities;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

public class VisionManager {
    private final AprilTagProcessor atProcessor;
    private final VisionPortal atPortal;
    private final ColorBlobLocatorProcessor ballProcessor;
    private final VisionPortal ballPortal;
    public final HiveState redHive;
    public final HiveState blueHive;
    /// Pitch of view above horizon, 0 = horizon, 90 = straight up
    public final double CAMERA_PITCH = 45;
    /// Height of camera sensor/lens above floor in inches
    public final double CAMERA_HEIGHT = 12;

    // These are measured from the OnShape models
    public final double HINGE_TO_TAG = 14.347493;
    public final double HINGE_TO_TAG_ANGLE = 5.993762;
    public final double HINGE_HEIGHT = 43.949700;


    public VisionManager(HardwareMap hardwareMap) {
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        atProcessor = AprilTagProcessor.easyCreateWithDefaults();
        atProcessor.setDecimation(1);
        WebcamName atCamera = hardwareMap.get(WebcamName.class, "C270");
        atPortal = new VisionPortal.Builder()
                .setCamera(atCamera)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(atProcessor)
                .setLiveViewContainerId(viewIds[0])
                .build();

        ballProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, new Scalar(20, 135, 75), new Scalar(240, 150, 85)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBlurSize(4)
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(0xFFFFFF00)
                .build();
        WebcamName ballCamera = hardwareMap.get(WebcamName.class, "C920");
        ballPortal = new VisionPortal.Builder()
                .setCamera(ballCamera)
                .setCameraResolution(new Size(320, 180))
                .addProcessor(ballProcessor)
                .setLiveViewContainerId(viewIds[1])
                .build();

        redHive = new HiveState(-30);
        blueHive = new HiveState(30);
    }

    public void update(Telemetry telemetry) {

        var atDetections = atProcessor.getDetections();
        telemetry.addData("AprilTags detected:", atDetections.size());
        var ballDetections = ballProcessor.getBlobs();
        telemetry.addData("Balls detected:", ballDetections.size());
        blueHive.current = false;
        redHive.current = false;
        // Keep track of distance used to calculate hive pos. If a tag is closer, then it's probably
        // more accurate, and the calculation should be overwritten
        var redHiveDistance = Double.POSITIVE_INFINITY;
        var blueHiveDistance = Double.POSITIVE_INFINITY;
        for (var tag : atDetections) {
            if (tag instanceof AprilTagClusterDetection cluster) {
                var pose = tag.ftcPose;
                var isRed = cluster.metadata.name.startsWith("RED");
                var isAudience = cluster.metadata.name.endsWith("AUDIENCE");
                var lastDistance = isRed ? redHiveDistance : blueHiveDistance;
                if (pose.range > lastDistance) continue;
                if (isRed) {
                    redHiveDistance = pose.range;
                    redHive.current = true;
                } else {
                    blueHiveDistance = pose.range;
                    blueHive.current = true;
                }
                // Using this instead of ftcPose.range means that tags with the same
                // ftcPose.elevation also have the same distance, i.e. horizontal
                // distance from the camera (ftcPose.x) is not considered
                var yzRange = Math.sqrt(
                        Math.pow(cluster.ftcPose.y, 2) + Math.pow(cluster.ftcPose.z, 2)
                );
                // "Elevation" refers to an angle here, ftcPose.elevation is degrees up from
                // center of camera view to tag
                var realElevation = CAMERA_PITCH + cluster.ftcPose.elevation;
                var tagHeight = CAMERA_HEIGHT + Math.sin(Math.toRadians(realElevation)) * yzRange;
                var hingeToTagAngle = Math.toDegrees(
                        Math.asin((tagHeight - HINGE_HEIGHT) / HINGE_TO_TAG)
                );
                // 0 = level, positive angle means this tag's cell is "pointed up"
                var hiveAngle = hingeToTagAngle + HINGE_TO_TAG_ANGLE;

                if (isAudience) hiveAngle *= -1;
                if (isRed) {
                    redHive.angle = hiveAngle;
                } else {
                    blueHive.angle = hiveAngle;
                }
            }
        }
        telemetry.addData("Blue hive angle:", String.valueOf(blueHive.angle));
        telemetry.addData("Red hive angle:", String.valueOf(redHive.angle));
    }

    /// REMEMBER TO CLOSE
    public void close() {
        atPortal.close();
        ballPortal.close();
    }

    /// Represents the detected position of a hive (the bar with two red or two blue cells)
    public static class HiveState {
        public double angle;
        /// Whether the robot saw the hive in the most current camera update.
        public boolean current = false;

        public HiveState(double angle) {
            this.angle = angle;
        }
    }
}
