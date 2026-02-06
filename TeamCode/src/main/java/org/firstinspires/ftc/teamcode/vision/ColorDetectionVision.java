package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Color Detection Vision", group = "Vision")
public class ColorDetectionVision extends LinearOpMode {

    OpenCvWebcam webcam;
    ColorDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        boolean cameraFound = false;
        
        // Initialize the webcam - try common names used in FTC configurations
        try {
            // Try different possible camera names based on typical FTC configurations
            webcam = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");
            cameraFound = true;
        } catch (IllegalArgumentException e) {
            try {
                webcam = hardwareMap.get(OpenCvWebcam.class, "Webcam 2");
                cameraFound = true;
            } catch (IllegalArgumentException e2) {
                try {
                    webcam = hardwareMap.get(OpenCvWebcam.class, "camera");
                    cameraFound = true;
                } catch (IllegalArgumentException e3) {
                    // If no webcam is found with common names, inform user
                    telemetry.addData("Error:", "No webcam found with common names.");
                    telemetry.addData("Info:", "Check your Robot Configuration.");
                    telemetry.addData("Expected Names:", "Webcam 1, Webcam 2, or camera");
                    telemetry.update();
                    sleep(5000); // Wait 5 seconds before continuing
                }
            }
        }
        
        if (cameraFound) {
            // Create and set the pipeline
            pipeline = new ColorDetectionPipeline(telemetry);
            webcam.setPipeline(pipeline);

            // Start streaming with the correct rotation
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Error:", "Camera error: " + errorCode);
                    telemetry.update();
                }
            });

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Instructions", "Press START to begin");
            telemetry.update();

            // Wait for the game to start
            waitForStart();

            // Main loop
            while (opModeIsActive()) {
                // The pipeline continuously updates telemetry with color information
                telemetry.update();
                
                // Allow some time for other processes
                sleep(20);
            }

            // Clean up when op mode ends
            webcam.stopStreaming();
        } else {
            // If no camera was found, still wait for start and then end
            waitForStart();
            while(opModeIsActive()) {
                telemetry.addData("Status", "No camera found - opmode ending");
                telemetry.update();
                sleep(100);
            }
        }
    }

    // Custom OpenCV pipeline for color detection
    static class ColorDetectionPipeline extends OpenCvPipeline {
        private Mat hsvMat = new Mat();
        private Mat roiMat = new Mat();
        private Mat outputMat = new Mat();
        
        private Telemetry telemetry;
        private Scalar rectColor = new Scalar(255, 0, 0); // Blue rectangle for ROI
        
        // Region of interest (ROI) - the area we'll analyze for color
        private Rect roi = new Rect(270, 190, 100, 100); // Center region of 640x480 image
        
        public ColorDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Clone the input frame to avoid modifying the original
            input.copyTo(outputMat);
            
            // Define the region of interest (ROI)
            roiMat = input.submat(roi);
            
            // Convert ROI from BGR to HSV
            Imgproc.cvtColor(roiMat, hsvMat, Imgproc.COLOR_RGB2HSV);
            
            // Calculate color statistics for the ROI
            calculateColorStats(hsvMat);
            
            // Draw rectangle around the ROI
            Imgproc.rectangle(outputMat, roi, rectColor, 2);
            
            return outputMat;
        }
        
        private void calculateColorStats(Mat hsvImage) {
            // Calculate average HSV values in the ROI
            Scalar avgHSV = Core.mean(hsvImage);
            
            // Extract individual channels (H, S, V)
            double[] hsvValues = avgHSV.val;
            double hue = hsvValues[0];           // Hue: 0-179 (OpenCV uses 0-179 for 0-360°)
            double saturation = hsvValues[1];    // Saturation: 0-255
            double value = hsvValues[2];         // Value (brightness): 0-255
            
            // Calculate RGB values as well
            Mat rgbMat = new Mat();
            Imgproc.cvtColor(roiMat, rgbMat, Imgproc.COLOR_RGB2BGR);
            Scalar avgRGB = Core.mean(rgbMat);
            double red = avgRGB.val[2];
            double green = avgRGB.val[1];
            double blue = avgRGB.val[0];
            
            // Calculate brightness metrics
            double brightness = (red + green + blue) / 3.0;
            double maxRGB = Math.max(Math.max(red, green), blue);
            double minRGB = Math.min(Math.min(red, green), blue);
            double lightness = (maxRGB + minRGB) / 2.0;
            double chroma = maxRGB - minRGB;
            
            // Display color information in telemetry
            telemetry.addData("=== COLOR ANALYSIS ===", "");
            telemetry.addData("Region", "Center 100x100 pixels");
            telemetry.addData("Coordinates", "(%d,%d) to (%d,%d)", roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);
            
            telemetry.addData("--- HSV Values ---", "");
            telemetry.addData("Hue (H)", "%.2f° (0-360° scale)", hue * 2); // Convert to 0-360°
            telemetry.addData("Saturation (S)", "%.2f (0-255)", saturation);
            telemetry.addData("Value (V)", "%.2f (0-255)", value);
            
            telemetry.addData("--- RGB Values ---", "");
            telemetry.addData("Red (R)", "%.2f (0-255)", red);
            telemetry.addData("Green (G)", "%.2f (0-255)", green);
            telemetry.addData("Blue (B)", "%.2f (0-255)", blue);
            
            telemetry.addData("--- Brightness/Lighting ---", "");
            telemetry.addData("Brightness", "%.2f (avg RGB)", brightness);
            telemetry.addData("Lightness", "%.2f ((max+min)/2)", lightness);
            telemetry.addData("Chroma", "%.2f (max-min)", chroma);
            
            // Estimate color name based on hue
            String colorName = estimateColorName(hue * 2); // Convert to 0-360°
            telemetry.addData("Estimated Color", colorName);
            
            // Color intensity indicators
            telemetry.addData("--- Color Intensity ---", "");
            telemetry.addData("Dominant Channel", getDominantChannel(red, green, blue));
            telemetry.addData("Saturation Level", getSaturationLevel(saturation));
            
            telemetry.update();
            
            // Release temporary matrices to prevent memory leaks
            rgbMat.release();
        }
        
        private String estimateColorName(double hue) {
            if (hue >= 330 || hue < 30) {
                return "Red";
            } else if (hue >= 30 && hue < 90) {
                return "Yellow";
            } else if (hue >= 90 && hue < 150) {
                return "Green";
            } else if (hue >= 150 && hue < 210) {
                return "Cyan";
            } else if (hue >= 210 && hue < 270) {
                return "Blue";
            } else if (hue >= 270 && hue < 330) {
                return "Magenta";
            }
            return "Unknown";
        }
        
        private String getDominantChannel(double r, double g, double b) {
            if (r >= g && r >= b) {
                return "Red";
            } else if (g >= r && g >= b) {
                return "Green";
            } else {
                return "Blue";
            }
        }
        
        private String getSaturationLevel(double s) {
            if (s < 50) {
                return "Low";
            } else if (s < 150) {
                return "Medium";
            } else {
                return "High";
            }
        }
        
        public void onViewportSizeChanged(int width, int height) {
            // Update ROI if viewport size changes
            roi = new Rect(width/2 - 50, height/2 - 50, 100, 100);
        }
    }
}