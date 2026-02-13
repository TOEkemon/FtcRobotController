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

@Autonomous(name = "Offset Test Opmode", group = "Testing")
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