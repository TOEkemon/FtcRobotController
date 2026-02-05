package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvPipeline;


import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;



import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.vision.VisionPortal;


/**
* BalldentifierAndDriver is an EasyOpenCV pipeline designed to detect and track
* purple and green balls for the DECODE game. It identifies the balls based on
* their color characteristics and provides their positions for the robot to interact with.
*/
public class BalldentifierAndDriver extends OpenCvPipeline {


   //hardware
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
   // Color thresholds for detecting purple and green balls
   // HSV color space is used for better color detection under varying lighting conditions


   // Purple ball HSV range
   private static final Scalar PURPLE_HSV_MIN = new Scalar(125, 50, 50);   // Lower HSV bounds for purple
   private static final Scalar PURPLE_HSV_MAX = new Scalar(150, 255, 255); // Upper HSV bounds for purple


   // Green ball HSV range
   private static final Scalar GREEN_HSV_MIN = new Scalar(40, 50, 50);     // Lower HSV bounds for green
   private static final Scalar GREEN_HSV_MAX = new Scalar(80, 255, 255);   // Upper HSV bounds for green


   // Internal Mats for processing
   private Mat hsvMat = new Mat();
   private Mat purpleMask = new Mat();
   private Mat greenMask = new Mat();
   private Mat combinedMask = new Mat();
   private Mat contoursOnFrameMat = new Mat();


   // Storage for contours and hierarchy
   private List<MatOfPoint> purpleContours = new ArrayList<>();
   private List<MatOfPoint> greenContours = new ArrayList<>();
   private List<MatOfPoint> allContours = new ArrayList<>();


   // Result storage
   private int purpleBallCount = 0;
   private int greenBallCount = 0;
   private int totalBallCount = 0;


   // Minimum area for a contour to be considered a ball
   private static final double MIN_BALL_AREA = 100;


   // Flag to control whether to draw contours on the output
   private boolean drawContours = true;






   /**
    * Main processing method called by EasyOpenCV for each camera frame
    * @param inputMat The input camera frame
    * @return The processed frame with overlays (contours, annotations)
    */
   @Override
   public Mat processFrame(Mat inputMat) {
       // Clone the input to avoid modifying the original
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


       // Find contours for both purple and green separately
       purpleContours.clear();
       greenContours.clear();
       allContours.clear();


       // Find contours in the purple mask
       Imgproc.findContours(purpleMask, purpleContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


       // Find contours in the green mask
       Imgproc.findContours(greenMask, greenContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


       // Count balls by filtering contours based on area
       purpleBallCount = 0;
       for (MatOfPoint contour : purpleContours) {
           double area = Imgproc.contourArea(contour);
           if (area > MIN_BALL_AREA) {
               purpleBallCount++;
               allContours.add(contour);
           }
       }


       greenBallCount = 0;
       for (MatOfPoint contour : greenContours) {
           double area = Imgproc.contourArea(contour);
           if (area > MIN_BALL_AREA) {
               greenBallCount++;
               allContours.add(contour);
           }
       }


       totalBallCount = purpleBallCount + greenBallCount;


       // Draw contours on the output frame if enabled
       if (drawContours) {
           // Draw purple contours in pink
           Imgproc.drawContours(contoursOnFrameMat, purpleContours, -1, new Scalar(255, 0, 255), 2);


           // Draw green contours in lime
           Imgproc.drawContours(contoursOnFrameMat, greenContours, -1, new Scalar(0, 255, 0), 2);


           // Add text overlay with ball count
           String text = "Purple: " + purpleBallCount + " Green: " + greenBallCount + " Total: " + totalBallCount;
           Imgproc.putText(contoursOnFrameMat, text, new Point(10, 30),
                          Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
       }


       // Release temporary mats to prevent memory leaks
       kernel.release();


       return contoursOnFrameMat;
   }


   /**
    * Gets the number of purple balls detected
    * @return Number of purple balls detected
    */
   public int getPurpleBallCount() {
       return purpleBallCount;
   }


   /**
    * Gets the number of green balls detected
    * @return Number of green balls detected
    */
   public int getGreenBallCount() {
       return greenBallCount;
   }


   /**
    * Gets the total number of balls detected
    * @return Total number of balls detected
    */
   public int getTotalBallCount() {
       return totalBallCount;
   }


   /**
    * Checks if any balls are detected
    * @return true if any balls are detected, false otherwise
    */
   public boolean areBallsDetected() {
       return totalBallCount > 0;
   }


   /**
    * Enables or disables drawing contours on the output frame
    * @param drawContours true to draw contours, false otherwise
    */
   public void setDrawContours(boolean drawContours) {
       this.drawContours = drawContours;
   }


   /**
    * Gets the current state of contour drawing
    * @return true if contours are being drawn, false otherwise
    */
   public boolean getDrawContours() {
       return drawContours;
   }


   /**
    * Gets the approximate horizontal position of the first detected ball
    * This can be used for alignment purposes in the autonomous routine
    * @return Normalized horizontal position (0.0 to 1.0) or -1 if no balls detected
    */
   public double getFirstBallHorizontalPosition() {
       if (totalBallCount == 0) {
           return -1; // No balls detected
       }


       // Find the first contour that meets the size criteria
       for (MatOfPoint contour : allContours) {
           double area = Imgproc.contourArea(contour);
           if (area > MIN_BALL_AREA) {
               // Calculate bounding rectangle to get position
               Rect rect = Imgproc.boundingRect(contour);
               // Return normalized horizontal position (0.0 to 1.0)
               return (double) rect.x / contoursOnFrameMat.width();
           }
       }


       return -1; // No valid contours found
   }


   /**
    * Gets the largest detected ball's position and size
    * Useful for prioritizing which ball to collect first
    * @return BallPositionInfo object containing position and size data
    */
   public BallPositionInfo getLargestBallInfo() {
       if (totalBallCount == 0) {
           return null; // No balls detected
       }


       double largestArea = 0;
       Rect largestRect = null;
       boolean isPurple = false;


       // Check purple contours
       for (MatOfPoint contour : purpleContours) {
           double area = Imgproc.contourArea(contour);
           if (area > largestArea && area > MIN_BALL_AREA) {
               largestArea = area;
               largestRect = Imgproc.boundingRect(contour);
               isPurple = true;
           }
       }


       // Check green contours
       for (MatOfPoint contour : greenContours) {
           double area = Imgproc.contourArea(contour);
           if (area > largestArea && area > MIN_BALL_AREA) {
               largestArea = area;
               largestRect = Imgproc.boundingRect(contour);
               isPurple = false;
           }
       }


       if (largestRect != null) {
           // Calculate normalized coordinates (0.0 to 1.0)
           double normX = (double) largestRect.x / contoursOnFrameMat.width();
           double normY = (double) largestRect.y / contoursOnFrameMat.height();
           double normWidth = (double) largestRect.width / contoursOnFrameMat.width();
           double normHeight = (double) largestRect.height / contoursOnFrameMat.height();


           return new BallPositionInfo(normX, normY, normWidth, normHeight, isPurple, largestArea);
       }


       return null;
   }


   /**
    * Helper class to store ball position and size information
    */
   public static class BallPositionInfo {
       public final double normalizedX;      // X position normalized (0.0 to 1.0)
       public final double normalizedY;      // Y position normalized (0.0 to 1.0)
       public final double normalizedWidth;  // Width normalized (0.0 to 1.0)
       public final double normalizedHeight; // Height normalized (0.0 to 1.0)
       public final boolean isPurple;        // True if purple, false if green
       public final double area;             // Area in pixels


       public BallPositionInfo(double normX, double normY, double normWidth, double normHeight, boolean isPurple, double area) {
           this.normalizedX = normX;
           this.normalizedY = normY;
           this.normalizedWidth = normWidth;
           this.normalizedHeight = normHeight;
           this.isPurple = isPurple;
           this.area = area;
       }


       /**
        * Gets the position as a Point with normalized coordinates
        * @return Point with normalized X and Y coordinates
        */
       public Point getNormalizedPosition() {
           return new Point(normalizedX, normalizedY);
       }


       /**
        * Checks if the ball is on the left side of the frame
        * @return true if the ball is in the left half of the frame
        */
       public boolean isOnLeftSide() {
           return normalizedX < 0.5;
       }


       /**
        * Checks if the ball is on the right side of the frame
        * @return true if the ball is in the right half of the frame
        */
       public boolean isOnRightSide() {
           return normalizedX >= 0.5;
       }


       /**
        * Checks if the ball is in the top half of the frame
        * @return true if the ball is in the top half of the frame
        */
       public boolean isInTopHalf() {
           return normalizedY < 0.5;
       }


       /**
        * Checks if the ball is in the bottom half of the frame
        * @return true if the ball is in the bottom half of the frame
        */
       public boolean isInBottomHalf() {
           return normalizedY >= 0.5;
       }
   }
   public void driveToBall() {
       // Placeholder for driving logic to approach the detected ball
       double maxArea = 0; // Variable to track the largest contour area
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
               double centerX = moments.get_m10() / moments.get_m00();
               double  centerY = moments.get_m01() / moments.get_m00();
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
       while (intakeAreaRed >=128 && intakeAreaGreen >=128 && intakeAreaBlue >=128 && intakeAreaRed <=150 && intakeAreaGreen <=150 && intakeAreaBlue <=150) {
           intakeMotor.setPower(1);
           frontLeftMotor.setPower(.5);
           backLeftMotor.setPower(.5);
           frontRightMotor.setPower(.5);
           backRightMotor.setPower(.5);
           if (intakeAreaRed <128 || intakeAreaGreen <128 || intakeAreaBlue <128 || intakeAreaRed >150 || intakeAreaGreen >150 || intakeAreaBlue >150) {
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

















