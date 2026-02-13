package org.firstinspires.ftc.teamcode.variabletests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;  // Add this import
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;  // Add this import



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


// Import vision-related classes
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import java.util.List;
import java.util.ArrayList;

@Autonomous(name = "Offset Test OpMode", group = "Testing")

// Custom OpenCV Pipeline for ball detection
class BallDetectionPipeline extends OpenCvPipeline {
    // Color thresholds for detecting purple and green balls
    private static final Scalar PURPLE_HSV_MIN = new Scalar(125, 50, 50);
    private static final Scalar PURPLE_HSV_MAX = new Scalar(150, 255, 255);
    private static final Scalar GREEN_HSV_MIN = new Scalar(40, 50, 50);
    private static final Scalar GREEN_HSV_MAX = new Scalar(80, 255, 255);

    // Internal Mats for processing
    private Mat hsvMat = new Mat();
    private Mat purpleMask = new Mat();
    private Mat greenMask = new Mat();
    private Mat combinedMask = new Mat();
    private Mat contoursOnFrameMat = new Mat();

    // Storage for contours
    private List<MatOfPoint> purpleContours = new ArrayList<>();
    private List<MatOfPoint> greenContours = new ArrayList<>();
    private List<MatOfPoint> allContours = new ArrayList<>();

    // Result storage
    private int purpleBallCount = 0;
    private int greenBallCount = 0;
    private int totalBallCount = 0;

    // Minimum area for a contour to be considered a ball
    private static final double MIN_BALL_AREA = 100;

    @Override
    public Mat processFrame(Mat inputMat) {
        inputMat.copyTo(contoursOnFrameMat);

        // Convert RGB to HSV for better color detection
        Imgproc.cvtColor(inputMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create masks for purple and green balls
        Core.inRange(hsvMat, PURPLE_HSV_MIN, PURPLE_HSV_MAX, purpleMask);
        Core.inRange(hsvMat, GREEN_HSV_MIN, GREEN_HSV_MAX, greenMask);

        // Combine the masks to detect both colors
        Core.bitwise_or(purpleMask, greenMask, combinedMask);

        // Apply morphological operations to reduce noise
        Mat kernel = Mat.ones(3, 3, CvType.CV_8U);
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        // Clear previous contours
        purpleContours.clear();
        greenContours.clear();
        allContours.clear();

        // Find contours in the combined mask
        List<MatOfPoint> tempContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, tempContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by area and add to allContours
        for (MatOfPoint contour : tempContours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_BALL_AREA) {
                allContours.add(contour);
            }
        }

        // Count balls by filtering contours based on area
        purpleBallCount = 0;
        for (MatOfPoint contour : purpleContours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_BALL_AREA) {
                purpleBallCount++;
            }
        }

        greenBallCount = 0;
        for (MatOfPoint contour : greenContours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_BALL_AREA) {
                greenBallCount++;
            }
        }

        totalBallCount = allContours.size();

        // Draw contours on the output frame
        Imgproc.drawContours(contoursOnFrameMat, allContours, -1, new Scalar(255, 0, 255), 2);

        // Add text overlay with ball count
        String text = "Balls: " + totalBallCount;
        Imgproc.putText(contoursOnFrameMat, text, new Point(10, 30),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);

        // Release temporary mat to prevent memory leaks
        kernel.release();
        hierarchy.release();

        return contoursOnFrameMat;
    }

    public List<MatOfPoint> getAllContours() {
        return allContours;
    }

    public int getTotalBallCount() {
        return totalBallCount;
    }
}

public class offsetTest extends LinearOpMode{

    private WebcamName webcam;
    private DcMotorEx intakeMotor;
    private IMU imu;  // Added missing IMU declaration
    private OpenCvCamera camera;  // Added missing camera declaration
    private long startTime;  // Added missing startTime declaration
    private List<MatOfPoint> allContours;  // Added missing allContours declaration
    private double maxArea = 0;  // Added missing maxArea declaration
    private int centerX = 0;  // Added missing centerX declaration
    private int centerY = 0;  // Added missing centerY declaration


    private BallDetectionPipeline pipeline; // Standalone pipeline instead of BalldentifierAndDriver

    private void initializeHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void runOpMode() {

        initializeHardware();


        // Create our custom standalone pipeline
        pipeline = new BallDetectionPipeline();

        int camCenterX = 320;
        int camCenterY = 240;

        // Note: getAllContours() will return an empty list until the pipeline processes camera frames
        // The contours are populated when the camera feeds frames to the pipeline
        List<MatOfPoint> contours = new ArrayList<>(); // Initialize as empty list

        int camViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());




        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewId);


        camera.setPipeline(pipeline);


        // Start camera streaming BEFORE waitForStart()
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(camCenterX*2, camCenterY*2);
            }


            @Override
            public void onError(int errorCode) {}
        });

        // Wait for start
        waitForStart();
        startTime = System.currentTimeMillis();

        // Main loop - continuously process and display contour information
        while (opModeIsActive()) {
            // Get contours from the pipeline (these are populated by the OpenCV pipeline as frames are processed)
            allContours = pipeline.getAllContours();

            MatOfPoint largestContour = null; // Variable to store the largest contour found
            maxArea = 0; // Reset max area for this iteration
            
            // Iterate through all contours to find the one with the largest area (closest ball)
            for (MatOfPoint contour : allContours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            
            // If a valid contour is found, calculate its center position
            if (largestContour != null) {
                Moments moments = Imgproc.moments(largestContour);
                if (moments.get_m00() != 0) {
                    centerX =  (int) ( moments.get_m10() / moments.get_m00());
                    centerY = (int) (moments.get_m01() / moments.get_m00());
                }
            }
            
            double currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);

            telemetry.addData("centerX: ", centerX);
            telemetry.addData("centerY: ", centerY);
            telemetry.addData("camCenterX: ", camCenterX);
            telemetry.addData("camCenterY: ", camCenterY);
            telemetry.addData("Max Area: ", maxArea);
            telemetry.addData("Contour Count: ", allContours.size());
            telemetry.addData("Current Draw", currentDraw);
            telemetry.update();
            
            sleep(50); // Small delay to allow other processes to run
        }
    }




}