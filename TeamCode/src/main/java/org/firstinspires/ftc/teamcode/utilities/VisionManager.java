package org.firstinspires.ftc.teamcode.utilities;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
    public final double CAMERA_PITCH = Math.toDegrees(Math.atan2(1, 2));
    /// Height of camera sensor/lens above floor in inches
    public final double CAMERA_HEIGHT = 14.3046;
    /// Horizontal/lateral distance (to the right) from turret exit point to camera
    public final double CAMERA_SHIFT = 3.48622;

    // These are measured from the OnShape models
    public final double HINGE_TO_TAG_FIELD_POS = 21.849157;
    public final double HINGE_TO_TAG_FIELD_POS_ANGLE = -10.879321;
    public final double HINGE_HEIGHT = 43.949700;


    public VisionManager(HardwareMap hardwareMap) {
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        atProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(
                        new Position(DistanceUnit.INCH, CAMERA_SHIFT, 0, CAMERA_HEIGHT, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, CAMERA_PITCH - 90, 0, 0)
                )
                .build();
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
        ballPortal.stopStreaming();

        redHive = new HiveState(-30);
        blueHive = new HiveState(30);
    }

    public void update(Telemetry telemetry) {

        var atDetections = atProcessor.getDetections();
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
                } else {
                    blueHiveDistance = pose.range;
                }
                var hive = isRed ? redHive : blueHive;
                var hiveAngle = 0;
                var tagHeight = 0;
                var horizontalDistance = 0;
                var bearing = -tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                if (isAudience) {
                    hiveAngle *= -1;
                    hive.audienceHeight = tagHeight;
                    hive.audienceDistance = horizontalDistance;
                    hive.audienceBearing = bearing;

                } else {
                    hive.scoringHeight = tagHeight;
                    hive.scoringDistance = horizontalDistance;
                    hive.scoringBearing = bearing;
                }
                hive.angle = hiveAngle;
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
    /// Distances are horizontal
    /// TODO: estimate initial values w/ starting team
    public static class HiveState {
        /// Follows right-hand rule for field axes.
        /// The blue hive starts at -30, red starts at +30
        public double angle;
        public double audienceBearing;
        public double audienceDistance;
        public double audienceHeight;
        public double scoringBearing;
        public double scoringDistance;
        public double scoringHeight;
        /// Whether the robot saw the hive in the most current camera update.
        public boolean current = false;

        public HiveState(double angle) {
            this.angle = angle;
        }
    }
}
