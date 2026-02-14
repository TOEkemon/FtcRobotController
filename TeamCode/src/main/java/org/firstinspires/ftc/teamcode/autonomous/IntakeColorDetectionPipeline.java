package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakeColorDetectionPipeline extends OpenCvPipeline {
    // Define the region of interest (ROI) for the intake area
    // These coordinates define the area where balls enter the intake
    private static final int ROI_X = 200;  // X coordinate of ROI top-left corner
    private static final int ROI_Y = 300;  // Y coordinate of ROI top-left corner
    private static final int ROI_WIDTH = 240;  // Width of ROI
    private static final int ROI_HEIGHT = 140; // Height of ROI
    
    // Color thresholds for detecting purple and green balls in HSV space
    private static final Scalar PURPLE_HSV_MIN = new Scalar(125, 50, 50);
    private static final Scalar PURPLE_HSV_MAX = new Scalar(150, 255, 255);
    private static final Scalar GREEN_HSV_MIN = new Scalar(40, 50, 50);
    private static final Scalar GREEN_HSV_MAX = new Scalar(80, 255, 255);
    
    // Internal Mats for processing
    private Mat hsvMat = new Mat();
    private Mat roiMat = new Mat();
    private Mat purpleMask = new Mat();
    private Mat greenMask = new Mat();
    private Mat combinedMask = new Mat();
    private Mat outputMat = new Mat();
    
    // Variables to store detection results
    private boolean ballDetected = false;
    private String detectedColor = "NONE"; // "PURPLE", "GREEN", or "NONE"
    private double confidence = 0.0; // Confidence level of detection
    
    @Override
    public Mat processFrame(Mat inputMat) {
        // Create a copy of the input for output
        inputMat.copyTo(outputMat);
        
        // Define the region of interest (ROI) where the intake is located
        Rect roi = new Rect(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT);
        
        // Extract the ROI from the input image
        roiMat = new Mat(inputMat, roi);
        
        // Convert ROI to HSV for better color detection
        Imgproc.cvtColor(roiMat, hsvMat, Imgproc.COLOR_RGB2HSV);
        
        // Create masks for purple and green colors
        Core.inRange(hsvMat, PURPLE_HSV_MIN, PURPLE_HSV_MAX, purpleMask);
        Core.inRange(hsvMat, GREEN_HSV_MIN, GREEN_HSV_MAX, greenMask);
        
        // Combine the masks
        Core.bitwise_or(purpleMask, greenMask, combinedMask);
        
        // Calculate the percentage of pixels that match the target colors
        double purplePercentage = (double) Core.countNonZero(purpleMask) / (roiMat.rows() * roiMat.cols());
        double greenPercentage = (double) Core.countNonZero(greenMask) / (roiMat.rows() * roiMat.cols());
        
        // Determine if a ball is detected based on color coverage
        double colorThreshold = 0.10; // 10% of pixels must be colored to detect a ball
        ballDetected = (purplePercentage > colorThreshold || greenPercentage > colorThreshold);
        
        // Determine the detected color based on which color has higher percentage
        if (ballDetected) {
            if (purplePercentage > greenPercentage) {
                detectedColor = "PURPLE";
                confidence = purplePercentage;
            } else {
                detectedColor = "GREEN";
                confidence = greenPercentage;
            }
        } else {
            detectedColor = "NONE";
            confidence = 0.0;
        }
        
        // Draw ROI rectangle on the output image
        Imgproc.rectangle(outputMat, roi.tl(), roi.br(), new Scalar(0, 255, 0), 2);
        
        // Add text overlay with detection results
        String detectionText = "Ball: " + (ballDetected ? detectedColor : "NO") + 
                             " (" + String.format("%.1f%%", confidence * 100) + ")";
        Imgproc.putText(outputMat, detectionText, new Point(10, 30),
                       Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
        
        // Release temporary mats to prevent memory leaks
        roiMat.release();
        hsvMat.release();
        purpleMask.release();
        greenMask.release();
        combinedMask.release();
        
        return outputMat;
    }
    
    public boolean isBallDetected() {
        return ballDetected;
    }
    
    public String getDetectedColor() {
        return detectedColor;
    }
    
    public double getConfidence() {
        return confidence;
    }
}