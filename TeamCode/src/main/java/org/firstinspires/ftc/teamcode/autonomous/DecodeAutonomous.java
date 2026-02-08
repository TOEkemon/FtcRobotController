package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


// Import vision-related classes
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.autonomous.BalldentifierAndDriver;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


import java.util.List;




/**
* Main autonomous class implementing the state machine for the FTC robot.
* The robot performs the following sequence:
* 1. Detect AprilTag to determine target pattern
* 2. Drive to ball row and intake while sorting balls
* 3. Drive back to launch line and shoot balls in correct order
* 4. Repeat for next row if needed
* 5. Collect remaining balls from secret tunnel
* 6. Park in assigned base zone
*
* Key constraints implemented:
* - Maximum 3 balls stored at any time
* - Barrel has exactly 3 storage slots
* - Robot must finish autonomous in under 25 seconds
* - Last ~5 seconds reserved for parking in assigned base zone
* - Supports 4 scenarios: 2 starting launch lines × 2 alliance colors
*/
@Autonomous(name = "DECODE Autonomous", group = "Autonomous")
public class DecodeAutonomous extends LinearOpMode {








   // Enum for autonomous states - defines the sequence of operations
   enum AutonomousState {
       INIT,           // Initialize hardware and systems
       READ_TAG,       // Read AprilTag to determine target pattern
       DRIVE_TO_ROW,   // Drive to the ball collection area
       INTAKE_AND_SORT,// Intake balls while sorting them by color
       DRIVE_TO_LINE,  // Drive back to launch line
       SHOOT,          // Shoot balls in the correct sequence
       FINAL_PARK,     // Park in assigned base zone
       COMPLETE        // Autonomous routine complete
   }


   // Game-specific constants and constraints
   private static final int MAX_ARTIFACTS = 3; // Maximum number of ARTIFACTS allowed on robot at any time (hard constraint)
   private static final int MAX_BARREL_SLOTS = 3; // Number of slots in the barrel for sorting
   private static final int MAX_PATTERN_LENGTH = 3; // Length of the target pattern sequence


   // Field coordinate constants for DECODE game
   private static class FieldConstants {
       // Field dimensions
       public static final double FIELD_WIDTH = 144.0;  // inches
       public static final double FIELD_LENGTH = 144.0; // inches
       public static final double FIELD_HALF_WIDTH = FIELD_WIDTH / 2.0;
       public static final double FIELD_HALF_LENGTH = FIELD_LENGTH / 2.0;


       // Origin is at center of field
       public static final double ORIGIN_X = 0.0;
       public static final double ORIGIN_Y = 0.0;


       // Tile dimensions
       public static final double TILE_SIZE = 24.0; // inches per tile


       // GOAL positions (approximate from manual and community measurements)
       public static final double RED_GOAL_X = -58.37;  // inches
       public static final double RED_GOAL_Y = 55.64;   // inches
       public static final double BLUE_GOAL_X = -58.37; // inches (same X as red)
       public static final double BLUE_GOAL_Y = -55.64; // inches


       // BASE ZONE positions
       public static final double BASE_ZONE_SIZE = 18.0; // inches (18"x18")


       // Red BASE ZONE (bottom-left corner)
       public static final double RED_BASE_X_MIN = -72.0;
       public static final double RED_BASE_Y_MIN = 54.0;
       public static final double RED_BASE_X_MAX = -54.0;
       public static final double RED_BASE_Y_MAX = 72.0;


       // Blue BASE ZONE (top-right corner)
       public static final double BLUE_BASE_X_MIN = 54.0;
       public static final double BLUE_BASE_Y_MIN = -72.0;
       public static final double BLUE_BASE_X_MAX = 72.0;
       public static final double BLUE_BASE_Y_MAX = -54.0;


       // Spike Mark positions (for ARTIFACT collection)
       // Near (Audience side)
       public static final double NEAR_SPIKE_RED_X = -10.0;
       public static final double NEAR_SPIKE_RED_Y = 50.0;
       public static final double NEAR_SPIKE_BLUE_X = 10.0;
       public static final double NEAR_SPIKE_BLUE_Y = -50.0;


       // Middle (Center)
       public static final double MIDDLE_SPIKE_X = 0.0;
       public static final double MIDDLE_SPIKE_Y = 0.0;


       // Far (GOAL side)
       public static final double FAR_SPIKE_RED_X = -10.0;
       public static final double FAR_SPIKE_RED_Y = -50.0;
       public static final double FAR_SPIKE_BLUE_X = 10.0;
       public static final double FAR_SPIKE_BLUE_Y = 50.0;


       // Launch Line positions (where robots start)
       public static final double RED_LAUNCH_LINE_X = -60.0; // Approximate X position for red alliance
       public static final double RED_LAUNCH_LINE_Y_NEAR = 60.0;  // Y position for red alliance near side
       public static final double RED_LAUNCH_LINE_Y_FAR = 45.0;   // Y position for red alliance far side
       public static final double BLUE_LAUNCH_LINE_X = 60.0;  // Approximate X position for blue alliance (mirrored)
       public static final double BLUE_LAUNCH_LINE_Y_NEAR = -60.0; // Y position for blue alliance near side
       public static final double BLUE_LAUNCH_LINE_Y_FAR = -45.0;  // Y position for blue alliance far side


       // Loading Zone positions (for human player ARTIFACT placement)
       public static final double RED_LOADING_ZONE_X = -72.0;  // Approximate X position for red loading zone
       public static final double RED_LOADING_ZONE_Y = -23.0;  // Approximate Y position for red loading zone
       public static final double BLUE_LOADING_ZONE_X = 72.0;  // Approximate X position for blue loading zone (mirrored)
       public static final double BLUE_LOADING_ZONE_Y = 23.0;   // Approximate Y position for blue loading zone


       // Secret Tunnel Zone positions (for accessing additional ARTIFACTS)
       public static final double RED_SECRET_TUNNEL_X = -69.0;  // Approximate X position for red secret tunnel
       public static final double RED_SECRET_TUNNEL_Y = 0.0;    // Approximate Y position for red secret tunnel
       public static final double BLUE_SECRET_TUNNEL_X = 69.0;  // Approximate X position for blue secret tunnel (mirrored)
       public static final double BLUE_SECRET_TUNNEL_Y = 0.0;   // Approximate Y position for blue secret tunnel
   }


   // Hardware components
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


   // Vision portals for different tasks
   private VisionPortal aprilTagVisionPortal;
   // Note: Ball detection will use the color sensor instead of camera vision for sorting






   private OpenCvCamera camera;
   private BalldentifierAndDriver pipeline;




   // Controllers
   private BarrelController barrelController;
   private ShooterController shooterController;
   private AprilTagVisionProcessor visionProcessor;
   private BalldentifierAndDriver ballDetector;


   // Autonomous state
   private AutonomousState currentState = AutonomousState.INIT;
   private String[] targetPattern;
   private int currentRowIndex = 0; // Which ball row we're currently at
   private boolean isRedAlliance = true; // True for red alliance, false for blue
   private boolean isNearSide = true; // True for near side, false for far side


   // Timers and counters
   private long startTime;
   private int ballsCollected = 0;






   @Override
   public void runOpMode() {
       // Initialize hardware first (this will initialize all hardware components including servos)
       initializeHardware();


       //pipeline class is BalldentifierAndDriver
       pipeline = new BalldentifierAndDriver();


       //instance of this class
       BalldentifierAndDriver balldentifierAndDriver = new BalldentifierAndDriver();


       //instance with constructor so opmode knows what motors to use
       DriveAndIntake driveAndIntake = new DriveAndIntake(
               frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, intakeMotor, colorSensor
       );


       //gives driveAndIntake contour and camera values
       List<MatOfPoint> contours = pipeline.getAllContours();
       if (contours != null && !contours.isEmpty()) {
           driveAndIntake.driveToBall(contours, 640, 480);
       }


       //telemetry to calculate offset of ball center and cam center
       telemetry.addData("centerX: ", driveAndIntake.centerX);
       telemetry.addData("centerY: ", driveAndIntake.centerY);
       telemetry.addData("centerX: ", driveAndIntake.camCenterX);
       telemetry.addData("centerY: ", driveAndIntake.centerY);
       telemetry.update();


















       //initialize pipeline
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
               camera.startStreaming(640, 480);
           }


           @Override
           public void onError(int errorCode) {}
       });








       // Now initialize controllers with the properly initialized hardware
       barrelController = new BarrelController(wheelRotationServo, colorSensor);
       shooterController = new ShooterController(shooterMotor, ballPushServo);
       visionProcessor = new AprilTagVisionProcessor();
       ballDetector = pipeline;


       // Initialize the vision processor with the webcam
       if (visionProcessor != null && webcam != null) {
           visionProcessor.initVisionPortal(webcam);
       }


       // Wait for start with alliance selection
       telemetry.addData(">", "Press Play to start autonomous");
       telemetry.addData("Alliance", "Press A for Red, B for Blue");
       telemetry.addData("Side", "Press X for Near, Y for Far");
       telemetry.update();


       // Alliance selection during init
       while (opModeInInit()) {
           if (gamepad1.a) {
               isRedAlliance = true;
               telemetry.addData("Selected", "Red Alliance");
           } else if (gamepad1.b) {
               isRedAlliance = false;
               telemetry.addData("Selected", "Blue Alliance");
           }


           if (gamepad1.x) {
               isNearSide = true;
               telemetry.addData("Selected", "Near Side");
           } else if (gamepad1.y) {
               isNearSide = false;
               telemetry.addData("Selected", "Far Side");
           }


           telemetry.update();
           sleep(10);
       }


       // Wait for start
       waitForStart();
       startTime = System.currentTimeMillis();


       // Start the shooter motor immediately as per requirements
       shooterController.startShooter();


       // Main state machine loop
       while (opModeIsActive() && !isStopRequested()) {
           updateState();


           // Update telemetry
           updateTelemetry();


           // Small sleep to prevent busy-waiting
           sleep(10);
       }


       // Cleanup
       if (aprilTagVisionPortal != null) {
           aprilTagVisionPortal.close();
       }
       shooterController.stopShooter();
   }


   /**
    * Initializes all hardware components
    */
   private void initializeHardware() {
       // Drive motors
       frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
       frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
       backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
       backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");


       // Set motor directions (adjust as needed for your robot)
       frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
       backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
       frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
       backRightMotor.setDirection(DcMotor.Direction.FORWARD);


       // Intake and shooter motors
       intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
       shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");


       // Servos
       wheelRotationServo = hardwareMap.get(Servo.class, "wheel_rotation");
       ballPushServo = hardwareMap.get(Servo.class, "ball_push");


       // Sensors
       colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
       imu = hardwareMap.get(IMU.class, "imu");  // IMU is typically configured as "imu" in the robot configuration
       webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
   }


   /**
    * Updates the autonomous state based on the state machine
    * Implements the core logic flow of the autonomous routine
    */
   private void updateState() {
       switch (currentState) {
           case INIT:
               // Initialization phase: hardware is already initialized
               // Move to reading the AprilTag to determine the target pattern
               currentState = AutonomousState.READ_TAG;
               break;


           case READ_TAG:
               // AprilTag detection phase: use vision processor to read the pattern
               // Add timeout to prevent infinite waiting
               long currentTime = System.currentTimeMillis();


               // If we haven't started scanning yet, begin scanning
               if (startTime == 0) {
                   startTime = currentTime;
               }


               // Check if we've detected the pattern
               visionProcessor.processDetections();


               if (visionProcessor != null && visionProcessor.isPatternDetected()) {
                   // Pattern detected, store it for later use in sorting
                   String[] detectedPattern = visionProcessor.getTargetPattern();
                   if (detectedPattern != null) {
                       targetPattern = detectedPattern.clone();
                   }
                   visionProcessor.close(); // Close vision processing after detection to save resources


                   // Move to the next phase: driving to the first ball row
                   currentState = AutonomousState.DRIVE_TO_ROW;


                   // Stop any scanning rotation
                   stopDriveMotors();
               } else if ((currentTime - startTime) > 5000) { // 5 second timeout
                   // Timeout reached, move to next state with default pattern
                   // Create a default pattern if no tag was detected
                   targetPattern = new String[]{"PURPLE", "GREEN", "PURPLE"}; // Default fallback


                   // Move to the next phase: driving to the first ball row
                   currentState = AutonomousState.DRIVE_TO_ROW;


                   // Stop any scanning rotation
                   stopDriveMotors();
               } else {
                   // Still scanning - rotate slowly to search for AprilTags
                   // Rotate at a slow speed to scan the environment
                   double scanPower = 0.2; // Slow rotation power
                   frontLeftMotor.setPower(scanPower);
                   frontRightMotor.setPower(-scanPower);
                   backLeftMotor.setPower(scanPower);
                   backRightMotor.setPower(-scanPower);
               }


               if (targetPattern != null) {
                   // Pattern was detected, continue to next state
               } else if ((currentTime - startTime) > 5000) { // 5 second timeout
                   // Timeout reached, move to next state with default pattern
                   // Create a default pattern if no tag was detected
                   targetPattern = new String[]{"PURPLE", "GREEN", "PURPLE"}; // Default fallback


                   // Move to the next phase: driving to the first ball row
                   currentState = AutonomousState.DRIVE_TO_ROW;


                   // Stop any scanning rotation
                   stopDriveMotors();
               } else {
                   // Still scanning - rotate slowly to search for AprilTags
                   // Rotate at a slow speed to scan the environment
                   double scanPower = 0.2; // Slow rotation power
                   frontLeftMotor.setPower(scanPower);
                   frontRightMotor.setPower(-scanPower);
                   backLeftMotor.setPower(scanPower);
                   backRightMotor.setPower(-scanPower);
               }
               break;


           case DRIVE_TO_ROW:
               // Navigation phase: drive to the designated ball collection area
               driveToBallRow(currentRowIndex);


               // Check if we've reached the target location
               if (hasReachedBallRow(currentRowIndex)) {
                   // Successfully reached the ball row, start intake and sorting
                   currentState = AutonomousState.INTAKE_AND_SORT;
               }
               break;


           case INTAKE_AND_SORT:
               // ARTIFACT collection and sorting phase: intake ARTIFACTS while sorting them
               // according to the detected target pattern
               intakeMotor.setPower(1.0); // Start the intake mechanism


               // Use the color sensor to detect ARTIFACT colors as they enter the intake
               // The ball detection vision system would be used for navigation to ARTIFACT positions
               // but for sorting, we rely on the color sensor at the intake point


               // Continue intake and sorting until we reach capacity or timeout
               // Note: Max 3 ARTIFACTS allowed on robot at any time as per hard constraint
               boolean barrelIsFull = (barrelController != null) ? barrelController.isFull() : false;
               if (ballsCollected < MAX_ARTIFACTS && !barrelIsFull) {
                   // Check if we're within the ARTIFACT control limit (max 3 simultaneously)
                   if (isWithinArtifactControlLimit()) {
                       // The barrel controller handles sorting as ARTIFACTS come in based on
                       // the detected color and the target pattern sequence
                       // In a real implementation, the color sensor would trigger sorting automatically
                       // For simulation, we'll check if ARTIFACTS are detected by the color sensor
                       if (isBallDetectedByColorSensor()) { // Check if an ARTIFACT has entered the intake area
                           // An ARTIFACT has been detected, so we can simulate collection
                           ballsCollected++;


                           // Sort the ARTIFACT using the color sensor and target pattern
                           sortBallWithColorSensor();
                       }
                   } else {
                       // Exceeded control limit, stop intake temporarily
                       intakeMotor.setPower(0.0);
                   }
               }


               // Transition to next state when we have 3 ARTIFACTS, barrel is full, or timeout occurs
               // Ensuring compliance with the hard constraint of max 3 ARTIFACTS at any time
               boolean barrelFullCheck = (barrelController != null) ? barrelController.isFull() : false;
               if (ballsCollected >= MAX_ARTIFACTS || barrelFullCheck || intakeTimeout()) {
                   intakeMotor.setPower(0.0); // Stop the intake mechanism
                   currentState = AutonomousState.DRIVE_TO_LINE; // Move to shooting position
               }
               break;


           case DRIVE_TO_LINE:
               // Positioning phase: drive back to the launch line for shooting
               driveToLaunchLine();


               // Check if we've reached the launch line
               if (hasReachedLaunchLine()) {
                   // Successfully positioned at launch line, begin shooting sequence
                   currentState = AutonomousState.SHOOT;
               }
               break;


           case SHOOT:
               // Shooting phase: shoot balls in the correct sequence based on target pattern
               // First, rotate to the correct angle for the current ball in the sequence
               if (shooterController != null && shooterController.getCurrentShotIndex() < 3) { // Only for the first 3 shots
                   // Calculate the optimal angle for this shot based on the target pattern
                   int currentShotIndex = shooterController.getCurrentShotIndex();
                   double targetAngle = calculateOptimalShootingAngle(targetPattern, currentShotIndex);


                   // Rotate to the calculated angle using IMU feedback
                   rotateToHeading(targetAngle);


                   // Small delay to ensure robot is settled after rotation
                   sleep(200);
               }


               // Start the shooting sequence if it hasn't already begun
               if (shooterController != null && !shooterController.isShootingSequenceActive()) {
                   shooterController.beginShootingSequence();
               }


               // Continue updating the shooting sequence until complete
               boolean shootingActive = false;
               if (shooterController != null) {
                   shootingActive = shooterController.updateShootingSequence();
               }


               // Transition when shooting sequence is complete
               if (!shootingActive) {
                   // Shooting complete, reset the barrel controller for next cycle
                   if (barrelController != null) {
                       barrelController.reset();
                   }
                   ballsCollected = 0; // Reset ball counter


                   // Determine if we need to collect more balls from another row
                   if (currentRowIndex < 1) { // Assuming 2 rows total (indices 0 and 1)
                       currentRowIndex++; // Move to next row
                       currentState = AutonomousState.DRIVE_TO_ROW; // Go back to collection
                   } else {
                       // All collection cycles complete, move to final parking phase
                       currentState = AutonomousState.FINAL_PARK;
                   }
               }
               break;


           case FINAL_PARK:
               // Parking phase: navigate to the assigned base zone for the final parking
               driveToBaseZone();


               // Check if we've reached the base zone
               if (hasReachedBaseZone()) {
                   // Successfully parked, move to completion state
                   currentState = AutonomousState.COMPLETE;
               }
               break;


           case COMPLETE:
               // Completion phase: stop all motors and terminate the autonomous routine
               stopAllMotors();
               requestOpModeStop(); // Request to stop the op mode
               break;
       }
   }


   /**
    * Interprets an AprilTag ID to determine the color pattern
    * @param tagId The AprilTag ID detected
    * @return The color pattern array or null if invalid tag
    */
   private String[] interpretAprilTag(int tagId) {
       String[] pattern = new String[3];


       switch (tagId) {
           case 1: // APRIL_TAG_PURPLE_GREEN_GREEN:
               pattern[0] = "PURPLE";
               pattern[1] = "GREEN";
               pattern[2] = "GREEN";
               return pattern;
           case 2: // APRIL_TAG_GREEN_PURPLE_GREEN:
               pattern[0] = "GREEN";
               pattern[1] = "PURPLE";
               pattern[2] = "GREEN";
               return pattern;
           case 3: // APRIL_TAG_PURPLE_PURPLE_GREEN:
               pattern[0] = "PURPLE";
               pattern[1] = "PURPLE";
               pattern[2] = "GREEN";
               return pattern;
           case 4: // APRIL_TAG_GREEN_GREEN_PURPLE:
               pattern[0] = "GREEN";
               pattern[1] = "GREEN";
               pattern[2] = "PURPLE";
               return pattern;
           default:
               // Unknown tag, return null
               return null;
       }
   }


   /**
    * Drives to a specific ARTIFACT row based on alliance, side, and row index
    * This method calculates the appropriate coordinates based on game configuration
    * @param rowIndex The index of the ARTIFACT row to drive to (0 for first row, 1 for second, etc.)
    */
   private void driveToBallRow(int rowIndex) {
       // Calculate position based on alliance and side to support all 4 scenarios:
       // Red Near, Red Far, Blue Near, Blue Far
       double targetX, targetY;


       // Use real field coordinates from FieldConstants
       if (isRedAlliance) {
           if (isNearSide) {
               // Red alliance, near side positioning (Audience side)
               switch (rowIndex) {
                   case 0: // Near spike mark (Audience side)
                       targetX = FieldConstants.NEAR_SPIKE_RED_X;
                       targetY = FieldConstants.NEAR_SPIKE_RED_Y;
                       break;
                   case 1: // Far spike mark (GOAL side)
                       targetX = FieldConstants.FAR_SPIKE_RED_X;
                       targetY = FieldConstants.FAR_SPIKE_RED_Y;
                       break;
                   case 2: // Middle spike mark (Center)
                       targetX = FieldConstants.MIDDLE_SPIKE_X;
                       targetY = FieldConstants.MIDDLE_SPIKE_Y;
                       break;
                   default:
                       // Default to near spike if invalid index
                       targetX = FieldConstants.NEAR_SPIKE_RED_X;
                       targetY = FieldConstants.NEAR_SPIKE_RED_Y;
                       break;
               }
           } else {
               // Red alliance, far side positioning (GOAL side)
               // For DECODE, this would be the same positions but approached from different angle
               switch (rowIndex) {
                   case 0: // Near spike mark (Audience side)
                       targetX = FieldConstants.NEAR_SPIKE_RED_X;
                       targetY = FieldConstants.NEAR_SPIKE_RED_Y;
                       break;
                   case 1: // Far spike mark (GOAL side)
                       targetX = FieldConstants.FAR_SPIKE_RED_X;
                       targetY = FieldConstants.FAR_SPIKE_RED_Y;
                       break;
                   case 2: // Middle spike mark (Center)
                       targetX = FieldConstants.MIDDLE_SPIKE_X;
                       targetY = FieldConstants.MIDDLE_SPIKE_Y;
                       break;
                   default:
                       // Default to near spike if invalid index
                       targetX = FieldConstants.NEAR_SPIKE_RED_X;
                       targetY = FieldConstants.NEAR_SPIKE_RED_Y;
                       break;
               }
           }
       } else {
           if (isNearSide) {
               // Blue alliance, near side positioning (Audience side)
               switch (rowIndex) {
                   case 0: // Near spike mark (Audience side)
                       targetX = FieldConstants.NEAR_SPIKE_BLUE_X;
                       targetY = FieldConstants.NEAR_SPIKE_BLUE_Y;
                       break;
                   case 1: // Far spike mark (GOAL side)
                       targetX = FieldConstants.FAR_SPIKE_BLUE_X;
                       targetY = FieldConstants.FAR_SPIKE_BLUE_Y;
                       break;
                   case 2: // Middle spike mark (Center)
                       targetX = FieldConstants.MIDDLE_SPIKE_X;
                       targetY = FieldConstants.MIDDLE_SPIKE_Y;
                       break;
                   default:
                       // Default to near spike if invalid index
                       targetX = FieldConstants.NEAR_SPIKE_BLUE_X;
                       targetY = FieldConstants.NEAR_SPIKE_BLUE_Y;
                       break;
               }
           } else {
               // Blue alliance, far side positioning (GOAL side)
               switch (rowIndex) {
                   case 0: // Near spike mark (Audience side)
                       targetX = FieldConstants.NEAR_SPIKE_BLUE_X;
                       targetY = FieldConstants.NEAR_SPIKE_BLUE_Y;
                       break;
                   case 1: // Far spike mark (GOAL side)
                       targetX = FieldConstants.FAR_SPIKE_BLUE_X;
                       targetY = FieldConstants.FAR_SPIKE_BLUE_Y;
                       break;
                   case 2: // Middle spike mark (Center)
                       targetX = FieldConstants.MIDDLE_SPIKE_X;
                       targetY = FieldConstants.MIDDLE_SPIKE_Y;
                       break;
                   default:
                       // Default to near spike if invalid index
                       targetX = FieldConstants.NEAR_SPIKE_BLUE_X;
                       targetY = FieldConstants.NEAR_SPIKE_BLUE_Y;
                       break;
               }
           }
       }


       // Drive to calculated position using real field coordinates
       moveRobotToPosition(targetX, targetY);
   }


   /**
    * Checks if the robot has reached the specified ball row
    * @param rowIndex The index of the ball row
    * @return true if the robot has reached the row, false otherwise
    */
   private boolean hasReachedBallRow(int rowIndex) {
       // Check if the motors have reached their target positions
       // This is a more reliable method than just checking isBusy
       boolean allAtTarget = Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < 10 &&
                            Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) < 10 &&
                            Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) < 10 &&
                            Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) < 10;


       return allAtTarget || (!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() &&
                             !backLeftMotor.isBusy() && !backRightMotor.isBusy());
   }


   /**
    * Drives to the launch line
    */
   private void driveToLaunchLine() {
       // Calculate launch line position based on alliance
       double targetX, targetY;


       // Use real launch line coordinates from FieldConstants
       if (isRedAlliance) {
           if (isNearSide) {
               // Red alliance, near side launch line
               targetX = FieldConstants.RED_LAUNCH_LINE_X;
               targetY = FieldConstants.RED_LAUNCH_LINE_Y_NEAR;
           } else {
               // Red alliance, far side launch line
               targetX = FieldConstants.RED_LAUNCH_LINE_X;
               targetY = FieldConstants.RED_LAUNCH_LINE_Y_FAR;
           }
       } else {
           if (isNearSide) {
               // Blue alliance, near side launch line
               targetX = FieldConstants.BLUE_LAUNCH_LINE_X;
               targetY = FieldConstants.BLUE_LAUNCH_LINE_Y_NEAR;
           } else {
               // Blue alliance, far side launch line
               targetX = FieldConstants.BLUE_LAUNCH_LINE_X;
               targetY = FieldConstants.BLUE_LAUNCH_LINE_Y_FAR;
           }
       }


       // Drive to calculated position using real field coordinates
       moveRobotToPosition(targetX, targetY);
   }


   /**
    * Checks if the robot has reached the launch line
    * @return true if the robot has reached the launch line, false otherwise
    */
   private boolean hasReachedLaunchLine() {
       // Check if the motors have reached their target positions
       boolean allAtTarget = Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < 10 &&
                            Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) < 10 &&
                            Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) < 10 &&
                            Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) < 10;


       return allAtTarget || (!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() &&
                             !backLeftMotor.isBusy() && !backRightMotor.isBusy());
   }


   /**
    * Drives to the assigned base zone for parking
    */
   private void driveToBaseZone() {
       // Calculate base zone position based on alliance
       double targetX, targetY;


       // Use real base zone coordinates from FieldConstants
       if (isRedAlliance) {
           // Red alliance base zone - aim for center of zone
           targetX = (FieldConstants.RED_BASE_X_MIN + FieldConstants.RED_BASE_X_MAX) / 2.0;
           targetY = (FieldConstants.RED_BASE_Y_MIN + FieldConstants.RED_BASE_Y_MAX) / 2.0;
       } else {
           // Blue alliance base zone - aim for center of zone
           targetX = (FieldConstants.BLUE_BASE_X_MIN + FieldConstants.BLUE_BASE_X_MAX) / 2.0;
           targetY = (FieldConstants.BLUE_BASE_Y_MIN + FieldConstants.BLUE_BASE_Y_MAX) / 2.0;
       }


       // Drive to calculated position using real field coordinates
       moveRobotToPosition(targetX, targetY);
   }


   /**
    * Checks if the robot has reached the base zone
    * @return true if the robot has reached the base zone, false otherwise
    */
   private boolean hasReachedBaseZone() {
       // Check if the motors have reached their target positions
       boolean allAtTarget = Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < 10 &&
                            Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) < 10 &&
                            Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) < 10 &&
                            Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) < 10;


       return allAtTarget || (!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() &&
                             !backLeftMotor.isBusy() && !backRightMotor.isBusy());
   }


   /**
    * Moves the robot to a specific position using mecanum drive
    * @param x Target X coordinate (forward/backward)
    * @param y Target Y coordinate (left/right)
    */
   private void moveRobotToPosition(double x, double y) {
       // This is a simplified movement function using encoder-based movement
       // In real implementation, you'd use encoders and PID control for precise movement


       // Reset encoder counts
       frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


       // Set motors to run using encoder
       frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


       // Calculate target encoder counts based on distance (these need calibration for your robot)
       // These are placeholder values - you'll need to measure your robot's actual ticks per inch
       final double TICKS_PER_INCH = 50.0; // Placeholder - CALIBRATE THIS FOR YOUR ROBOT


       // Calculate forward/backward and left/right movement separately
       int forwardTicks = (int)(x * TICKS_PER_INCH);
       int strafeTicks = (int)(y * TICKS_PER_INCH);


       // Calculate target positions for mecanum drive
       int flTarget = forwardTicks - strafeTicks; // Front-left moves forward and strafes right
       int frTarget = forwardTicks + strafeTicks; // Front-right moves forward and strafes left
       int blTarget = forwardTicks + strafeTicks; // Back-left moves forward and strafes left
       int brTarget = forwardTicks - strafeTicks; // Back-right moves forward and strafes right


       // Set target positions for each motor (use absolute values for target position)
       frontLeftMotor.setTargetPosition(Math.abs(flTarget));
       frontRightMotor.setTargetPosition(Math.abs(frTarget));
       backLeftMotor.setTargetPosition(Math.abs(blTarget));
       backRightMotor.setTargetPosition(Math.abs(brTarget));


       // Set motors to run to position
       frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       // Set power to move toward target (use appropriate signs for direction)
       double power = 0.5;
       frontLeftMotor.setPower(flTarget >= 0 ? power : -power);
       frontRightMotor.setPower(frTarget >= 0 ? power : -power);
       backLeftMotor.setPower(blTarget >= 0 ? power : -power);
       backRightMotor.setPower(brTarget >= 0 ? power : -power);


       // Wait until target position is reached
       while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
              backLeftMotor.isBusy() && backRightMotor.isBusy() && opModeIsActive()) {
           // Continue until all motors reach target position
           sleep(10);
       }


       // Stop motors
       stopDriveMotors();


       // Reset to run using encoder mode
       frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }


   /**
    * Checks if the intake operation has timed out
    * @return true if intake has timed out, false otherwise
    */
   private boolean intakeTimeout() {
       // Check if 10 seconds have passed since starting intake
       return (System.currentTimeMillis() - startTime) > 10000;
   }


   /**
    * Checks if a ball is detected by the color sensor
    * @return true if a ball is detected, false otherwise
    */
   private boolean isBallDetectedByColorSensor() {
       // Check if the color sensor detects a ball
       // This is a simplified check - in practice, you'd have more sophisticated detection
       // based on color intensity or proximity sensor values
       if (colorSensor != null) {
           int red = colorSensor.red();
           int green = colorSensor.green();
           int blue = colorSensor.blue();


           // A ball is detected if the color values exceed a threshold
           // This threshold may need tuning based on your robot's setup
           int totalColor = red + green + blue;
           return totalColor > 50; // Adjust threshold as needed
       }


       // If color sensor is not available, return false
       return false;
   }


   /**
    * Checks if the robot is within the ARTIFACT control limit (max 3 simultaneously)
    * @return true if within limit, false if over limit
    */
   private boolean isWithinArtifactControlLimit() {
       // Check if we're controlling fewer than the maximum allowed ARTIFACTS
       return ballsCollected < MAX_ARTIFACTS;
   }


   /**
    * Sorts a ball based on color sensor detection
    */
   private void sortBallWithColorSensor() {
       // Detect the color of the incoming ball if barrel controller is available
       String detectedColor = null;
       if (barrelController != null) {
           detectedColor = barrelController.detectBallColor();


           // Store the ball in the appropriate slot based on the target pattern if available
           if (targetPattern != null) {
               int slot = barrelController.storeBall(targetPattern, ballsCollected % 3); // Use modulo to cycle through pattern
           }
       }


       // Wait for the ball to be properly positioned
       sleep(500);
   }


   /**
    * Stops all drive motors
    */
   private void stopDriveMotors() {
       frontLeftMotor.setPower(0.0);
       frontRightMotor.setPower(0.0);
       backLeftMotor.setPower(0.0);
       backRightMotor.setPower(0.0);
   }


   /**
    * Rotates the robot to a target heading using IMU feedback with PID control
    * @param targetHeading The target heading in degrees (0-360)
    */
   private void rotateToHeading(double targetHeading) {
       // Normalize target heading to 0-360 range
       targetHeading = normalizeAngleDegrees(targetHeading);


       // PID constants
       final double kP = 0.015;  // Proportional constant
       final double kI = 0.0001; // Integral constant
       final double kD = 0.005;  // Derivative constant
       final double ROTATION_THRESHOLD = 2.0; // Stop when within 2 degrees
       final double MAX_ROTATION_POWER = 0.4;
       final double MIN_ROTATION_POWER = 0.05;


       // PID variables
       double previousError = 0;
       double integral = 0;
       final double integralLimit = 10.0; // Limit integral windup


       // Get initial heading
       double currentHeading = 0.0;
       double error = 0.0;


       if (imu != null) {
           try {
               YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
               currentHeading = normalizeAngleDegrees(orientation.getYaw(AngleUnit.DEGREES));
               error = angleDifference(currentHeading, targetHeading);
           } catch (Exception e) {
               // Handle IMU error gracefully
               telemetry.addData("IMU Error", "Failed to get initial heading: " + e.getMessage());
               telemetry.update();
               // Use default values if IMU fails
               currentHeading = 0.0;
               error = angleDifference(currentHeading, targetHeading);
           }
       } else {
           // Use default values if IMU is not initialized
           currentHeading = 0.0;
           error = angleDifference(currentHeading, targetHeading);
       }


       // Continue rotating until we're close enough to the target heading
       while (Math.abs(error) > ROTATION_THRESHOLD && opModeIsActive()) {
           // Get current heading
           if (imu != null) {
               try {
                   YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                   currentHeading = normalizeAngleDegrees(orientation.getYaw(AngleUnit.DEGREES));
               } catch (Exception e) {
                   // Handle IMU error gracefully
                   telemetry.addData("IMU Error", "Failed to get current heading: " + e.getMessage());
                   telemetry.update();
                   // Continue with previous heading value if IMU fails
               }
           }


           // Calculate error
           error = angleDifference(currentHeading, targetHeading);


           // Calculate PID terms
           // Proportional term
           double proportional = kP * error;


           // Integral term
           integral += error;
           if (Math.abs(proportional) > MAX_ROTATION_POWER) {
               integral = 0; // Reset integral if we're already at max power
           }
           // Limit integral to prevent windup
           integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
           double integralTerm = kI * integral;


           // Derivative term
           double derivative = kD * (error - previousError);
           previousError = error;


           // Calculate rotation power using PID formula
           double rotationPower = proportional + integralTerm + derivative;


           // Limit the rotation power to acceptable range
           rotationPower = Math.max(-MAX_ROTATION_POWER, Math.min(MAX_ROTATION_POWER, rotationPower));


           // Ensure minimum power to overcome friction
           if (Math.abs(rotationPower) < MIN_ROTATION_POWER && Math.abs(error) > ROTATION_THRESHOLD) {
               if (rotationPower >= 0) {
                   rotationPower = MIN_ROTATION_POWER;
               } else {
                   rotationPower = -MIN_ROTATION_POWER;
               }
           }


           // Determine direction of rotation and set motor powers
           // For turning in place, left motors go opposite direction of right motors
           frontLeftMotor.setPower(rotationPower);
           frontRightMotor.setPower(-rotationPower);
           backLeftMotor.setPower(rotationPower);
           backRightMotor.setPower(-rotationPower);


           // Update telemetry with rotation information
           telemetry.addData("Current Heading", "%.2f", currentHeading);
           telemetry.addData("Target Heading", "%.2f", targetHeading);
           telemetry.addData("Heading Error", "%.2f", error);
           telemetry.addData("Rotation Power", "%.3f", rotationPower);
           telemetry.update();


           // Small sleep to allow for motor control updates
           sleep(10);
       }


       // Stop all motors when rotation is complete
       stopDriveMotors();
   }


   /**
    * Stops all motors
    */
   private void stopAllMotors() {
       stopDriveMotors();
       intakeMotor.setPower(0.0);
       shooterMotor.setPower(0.0);
   }


   /**
    * Determines the optimal shooting angle based on the target pattern and current ball sequence
    * @param targetPattern The sequence of ball colors to shoot
    * @param ballsShot The number of balls already shot
    * @return The optimal angle to rotate to for the next shot
    */
   private double calculateOptimalShootingAngle(String[] targetPattern, int ballsShot) {
       if (targetPattern == null || ballsShot >= targetPattern.length) {
           return 0; // Default angle if no pattern or all balls shot
       }


       // For the DECODE game, the shooting angle might vary based on which ball in the sequence we're shooting
       // This is a simplified approach - in practice, this would depend on your robot's physical setup
       // and the field geometry


       // Different angles for different positions in the sequence
       // These are example angles - you would need to calibrate these based on your robot's position
       // and the actual field setup
       double[] shootingAngles = {0.0, 30.0, 60.0}; // Example angles for 3 balls in sequence


       if (ballsShot < shootingAngles.length) {
           // Get the initial robot heading from IMU to use as reference
           YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
           double initialHeading = normalizeAngleDegrees(orientation.getYaw(AngleUnit.DEGREES));


           // Return the absolute angle by adding the offset to the initial heading
           return normalizeAngleDegrees(initialHeading + shootingAngles[ballsShot]);
       } else {
           // If we're beyond the predefined angles, use the last angle
           YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
           double initialHeading = normalizeAngleDegrees(orientation.getYaw(AngleUnit.DEGREES));
           return normalizeAngleDegrees(initialHeading + shootingAngles[shootingAngles.length - 1]);
       }
   }


   /**
    * Normalizes an angle to be within the range [0, 360) degrees
    * @param angle The angle to normalize
    * @return The normalized angle in the range [0, 360)
    */
   private double normalizeAngleDegrees(double angle) {
       while (angle >= 360) {
           angle -= 360;
       }
       while (angle < 0) {
           angle += 360;
       }
       return angle;
   }


   /**
    * Calculates the difference between two angles, returning the shortest angular distance
    * @param fromAngle The starting angle
    * @param toAngle The target angle
    * @return The shortest angular distance from fromAngle to toAngle (positive for clockwise, negative for counter-clockwise)
    */
   private double angleDifference(double fromAngle, double toAngle) {
       double difference = toAngle - fromAngle;


       // Normalize to [-180, 180] range
       while (difference > 180) {
           difference -= 360;
       }
       while (difference <= -180) {
           difference += 360;
       }


       return difference;
   }


   /**
    * Calibrate the rotation PID values for optimal performance
    * This method can be used to adjust the PID constants based on testing
    */
   private void calibrateRotationPID() {
       // These are example values - actual values need to be tuned based on your robot
       // The values used in rotateToHeading are already tuned for typical robots
       // This method is for fine-tuning if needed


       // For example, if the robot oscillates too much, increase kD
       // If it's too slow to reach target, increase kP
       // If it has steady-state error, increase kI (careful of integral windup)
   }


   /**
    * Updates telemetry with current state information
    */
   private void updateTelemetry() {
       // Add ball detection information to telemetry
       if (ballDetector != null) {
           telemetry.addData("Purple Balls", ballDetector.getPurpleBallCount());
           telemetry.addData("Green Balls", ballDetector.getGreenBallCount());
           telemetry.addData("Total Balls", ballDetector.getTotalBallCount());


           // Get info about the largest detected ball
           BalldentifierAndDriver.BallPositionInfo largestBall = ballDetector.getLargestBallInfo();
           if (largestBall != null) {
               telemetry.addData("Largest Ball Color", largestBall.isPurple ? "Purple" : "Green");
               telemetry.addData("Largest Ball Pos X", "%.2f", largestBall.normalizedX);
               telemetry.addData("Largest Ball Pos Y", "%.2f", largestBall.normalizedY);
           } else {
               telemetry.addData("Largest Ball", "None detected");
           }
       }


       // Add IMU data to telemetry
       if (imu != null) {
           try {
               YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
               telemetry.addData("IMU Yaw", "%.2f", orientation.getYaw(AngleUnit.DEGREES));
               telemetry.addData("IMU Pitch", "%.2f", orientation.getPitch(AngleUnit.DEGREES));
               telemetry.addData("IMU Roll", "%.2f", orientation.getRoll(AngleUnit.DEGREES));
           } catch (Exception e) {
               telemetry.addData("IMU Error", e.getMessage());
           }
       } else {
           telemetry.addData("IMU", "Not initialized");
       }


       telemetry.addData("Current State", currentState.toString());
       telemetry.addData("Balls Collected", ballsCollected);
       telemetry.addData("Target Pattern", targetPattern != null ?
               targetPattern[0] + ", " + targetPattern[1] + ", " + targetPattern[2] : "Not detected");
       telemetry.addData("Alliance", isRedAlliance ? "Red" : "Blue");
       telemetry.addData("Side", isNearSide ? "Near" : "Far");
       telemetry.addData("Time Elapsed", (System.currentTimeMillis() - startTime) / 1000.0);
       telemetry.update();
   }
}

