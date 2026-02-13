package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;  // Add this import
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;  // Add this import



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;


import java.util.List;




public class DriveAndIntake {


   private DcMotor frontLeftMotor;
   private DcMotor frontRightMotor;
   private DcMotor backLeftMotor;
   private DcMotor backRightMotor;
   private DcMotorEx intakeMotor;
   private DcMotor shooterMotor;
   private Servo wheelRotationServo;
   private Servo ballPushServo;
   private IMU imu;
   private WebcamName webcam;




   // Placeholder for driving logic to approach the detected ball
   double maxArea = 0; // Variable to track the largest contour area


   int camHeight =640;
   int camWidth = 480;


   int centerX = 0;
   int centerY = 0;


   int camCenterX = 320;
   int camCenterY = 240;



   // Use camera-based detection instead of color sensor
   // These values will be updated based on camera analysis in the main loop
   // For now, we'll use the camera pipeline's detection results

   //find offset trough physcial testing
   int offset = 39;


   double currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);

    public int getCenterX() {
        return centerX;
    }

   // contour is a MatOfPoint








   // Constructor
   /*public DriveAndIntake(
           DcMotor fl,
           DcMotor fr,
           DcMotor bl,
           DcMotor br,
           DcMotorEx intake
   ) {
       this.frontLeftMotor = fl;
       this.frontRightMotor = fr;
       this.backLeftMotor = bl;
       this.backRightMotor = br;
       this.intakeMotor = intake;
   }*/










   public void driveToBall(List<MatOfPoint> allContours, int camWidth, int camHeight) {





       double maxArea = 0;   // RESET EVERY FRAME
       MatOfPoint largestContour = null;

       Moments moments = Imgproc.moments(largestContour);




       // Variable to store the largest contour found
       // Iterate through all contours to find the one with the largest area (closest ball)
       for (MatOfPoint contour : allContours) {
           double area = Imgproc.contourArea(contour);
           if (area > maxArea) {
               maxArea = area;
               largestContour = contour;
           }
       }
       // If a valid contour is found, calculate its center position
       //redundancy just to make sure
       if (largestContour != null) {
           moments = Imgproc.moments(largestContour);
           if (moments.get_m00() != 0) {
               centerX =  (int) ( moments.get_m10() / moments.get_m00());
               centerY = (int) (moments.get_m01() / moments.get_m00());
           }
           else if (moments.get_m00() == 0) {
               centerX= 0;
               centerY = 0;
           }
       } else if (largestContour == null) {
           centerX = 0;
           centerY = 0;
       }
       currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);




       //find center of frame
       //IMPORTANT!!! webcam is offset by a few inches, must find offset
       //comment this out for simpler controlling in decodeAutonomous
       /*
       Point frameCenter = new Point(camWidth / 2, camHeight / 2);
       int camCenterX = camWidth / 2;
       //align ball's x value with frame's x value
       if (centerX != camCenterX) {
           frontLeftMotor.setPower(0.2);
           backLeftMotor.setPower(0.2);
           frontRightMotor.setPower(-0.2);
           backRightMotor.setPower(-0.2);
           if (centerX == camCenterX) {
               frontLeftMotor.setPower(0);
               backLeftMotor.setPower(0);
               frontRightMotor.setPower(0);
               backRightMotor.setPower(0);


           }
       }
       // move towards ball until it is taken in, which will be detected by current draw increase
       //when ball taken in, amp draw will increase, so use that as threshold to stop driving forward and stop intake
       intakeMotor.setPower(1);
       currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);
       if (currentDraw < 1.1) { // Adjust the threshold as needed based on testing
              currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);


              frontLeftMotor.setPower(0.5);
              backLeftMotor.setPower(0.5);
              frontRightMotor.setPower(0.5);
              backRightMotor.setPower(0.5);
         } else if (currentDraw >= 1.1) {

              intakeMotor.setPower(0);
              frontLeftMotor.setPower(0);
              backLeftMotor.setPower(0);
              frontRightMotor.setPower(0);
              backRightMotor.setPower(0);

           try {
               Thread.sleep( 1000);
           } catch (InterruptedException ie) {
               Thread.currentThread().interrupt();
           }
              intakeMotor.setPower(0);
    }
       /*while (intakeAreaRed >= 128 && intakeAreaGreen >= 128 && intakeAreaBlue >= 128 && intakeAreaRed <= 150 && intakeAreaGreen <= 150 && intakeAreaBlue <= 150) {
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
         }*/




       




   }
}
