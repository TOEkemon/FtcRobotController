package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.*;








public class DriveAndIntake {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor intakeMotor;
    private DcMotor shooterMotor;
    private Servo wheelRotationServo;
    private Servo ballPushServo;
    private ColorSensor colorSensor;
    private IMU imu;
    private WebcamName webcam;


    // Placeholder for driving logic to approach the detected ball
    double maxArea = 0; // Variable to track the largest contour area


    // contour is a MatOfPoint


    public void driveToBall(List<MatOfPoint> allContours, int camWidth, int camHeight) {

        double centerX = 0;
        double centerY = 0;


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
                centerX = moments.get_m10() / moments.get_m00();
                centerY = moments.get_m01() / moments.get_m00();
            }
        }
        //find center of frame
        Point frameCenter = new Point(camWidth / 2, camHeight / 2);
        int camCenterX = camWidth / 2;
        //align ball's x value with frame's x value
        while (centerX != camCenterX) {
            frontLeftMotor.setPower(0.2);
            backLeftMotor.setPower(0.2);
            frontRightMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            if (centerX == camCenterX) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            }
        }
        // move towards ball until it is taken in, which will be detected by color sensor in intake area not reading gray
        int intakeAreaRed = colorSensor.red();
        int intakeAreaGreen = colorSensor.green();
        int intakeAreaBlue = colorSensor.blue();
        while (intakeAreaRed >= 128 && intakeAreaGreen >= 128 && intakeAreaBlue >= 128 && intakeAreaRed <= 150 && intakeAreaGreen <= 150 && intakeAreaBlue <= 150) {
            intakeMotor.setPower(1);
            frontLeftMotor.setPower(.5);
            backLeftMotor.setPower(.5);
            frontRightMotor.setPower(.5);
            backRightMotor.setPower(.5);
            if (intakeAreaRed < 128 || intakeAreaGreen < 128 || intakeAreaBlue < 128 || intakeAreaRed > 150 || intakeAreaGreen > 150 || intakeAreaBlue > 150) {
                intakeMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            }


        }


    }
}