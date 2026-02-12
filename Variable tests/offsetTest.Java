package org.firstinspires.ftc.teamcode;

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
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.autonomous.BalldentifierAndDriver;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import java.util.List;

@Autonomous

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


    private BalldentifierAndDriver ballDetector;
    private BalldentifierAndDriver pipeline;

    private void initializeHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void runOpMode() {

        initializeHardware();


        //pipeline class is BalldentifierAndDriver
        pipeline = new BalldentifierAndDriver();


        //instance of this class
        BalldentifierAndDriver balldentifierAndDriver = new BalldentifierAndDriver();

        int camCenterX = 320;
        int camCenterY = 240;

        List<MatOfPoint> contours = pipeline.getAllContours();

        int camViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());




        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewId);


        camera.setPipeline(pipeline);


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

        // Main state machine loop - removed since updateState and updateTelemetry don't exist in this class
        // Just process the contours once for testing purposes
        allContours = pipeline.getAllContours();

        MatOfPoint largestContour = null; // Variable to store the largest contour found
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

        telemetry.addData("Current Draw", currentDraw);
        telemetry.update();
        sleep(10);
    }




}















