package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * BarrelController handles the barrel rotation and ball sorting logic.
 * The barrel has 3 storage slots controlled by a motor with encoder positions:
 * Slot 0 -> 0 encoder ticks
 * Slot 1 -> ~120 encoder ticks  
 * Slot 2 -> ~240 encoder ticks
 */
public class BarrelController {

    // Encoder positions for each slot (these need to be calibrated for your specific robot)
    // These values represent encoder ticks for each slot position
    private static final int SLOT_0_POSITION = 0;      // Starting position for slot 0
    private static final int SLOT_1_POSITION = 120;    // Position for slot 1 (calibrate as needed)
    private static final int SLOT_2_POSITION = 240;    // Position for slot 2 (calibrate as needed)

    // Motor configuration
    private static final double MOTOR_POWER = 0.3;  // Power to use when rotating the motor

    // Hardware components
    private DcMotor wheelRotationMotor;
    private ColorSensor colorSensor;

    // State tracking
    private int currentBallIndex = 0;  // Index of the next ball to store
    private boolean[] slotOccupied = {false, false, false};  // Track which slots have balls

    /**
     * Constructor for BarrelController
     * @param wheelRotationMotor The motor controlling barrel rotation
     * @param colorSensor The color sensor for detecting ball colors
     */
    public BarrelController(DcMotor wheelRotationMotor, ColorSensor colorSensor) {
        this.wheelRotationMotor = wheelRotationMotor;
        this.colorSensor = colorSensor;
        
        // Configure the motor for position control
        this.wheelRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.wheelRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wheelRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Rotates the barrel to the specified slot position using encoder control
     * @param slotNumber The slot number (0, 1, or 2)
     */
    public void rotateToSlot(int slotNumber) {
        if (slotNumber < 0 || slotNumber > 2) {
            throw new IllegalArgumentException("Slot number must be 0, 1, or 2");
        }

        // Set the target position based on the slot
        int targetPosition;
        switch (slotNumber) {
            case 0:
                targetPosition = SLOT_0_POSITION;
                break;
            case 1:
                targetPosition = SLOT_1_POSITION;
                break;
            case 2:
                targetPosition = SLOT_2_POSITION;
                break;
            default:
                targetPosition = SLOT_0_POSITION; // Default to slot 0
                break;
        }

        // Set the target position and run to position
        wheelRotationMotor.setTargetPosition(targetPosition);
        wheelRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRotationMotor.setPower(MOTOR_POWER);

        // Wait until the motor reaches the target position
        while (wheelRotationMotor.isBusy()) {
            // Wait for the motor to reach the target
        }

        // Stop the motor and switch back to RUN_USING_ENCODER mode
        wheelRotationMotor.setPower(0);
        wheelRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Gets the encoder position for a given slot
     * @param slotNumber The slot number (0, 1, or 2)
     * @return The encoder position for the slot
     */
    public int getSlotPosition(int slotNumber) {
        switch (slotNumber) {
            case 0: return SLOT_0_POSITION;
            case 1: return SLOT_1_POSITION;
            case 2: return SLOT_2_POSITION;
            default: throw new IllegalArgumentException("Slot number must be 0, 1, or 2");
        }
    }
    
    /**
     * Stores a ball in the appropriate slot based on the target pattern
     * @param targetPattern The ordered list of 3 colors to match
     * @param currentIndex The current index in the target pattern
     * @return The slot number where the ball was stored
     */
    public int storeBall(String[] targetPattern, int currentIndex) {
        if (currentBallIndex >= 3) {
            // Barrel is full, cannot store more balls
            return -1;
        }

        // Detect the color of the incoming ball
        String detectedColor = detectBallColor();

        // Find the appropriate slot based on the target pattern
        int targetSlot = findTargetSlot(targetPattern, currentIndex, detectedColor);

        // Rotate to the target slot
        rotateToSlot(targetSlot);

        // Mark the slot as occupied
        slotOccupied[targetSlot] = true;

        // Increment the ball index
        currentBallIndex++;

        return targetSlot;
    }
    
    /**
     * Detects the color of the ball using the color sensor
     * @return The detected color as a string ("PURPLE", "GREEN", etc.)
     */
    public String detectBallColor() {
        // This is a simplified color detection - in practice, you'd use the actual color values
        // from the color sensor to determine the color
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // For DECODE game, we need to detect purple and green balls
        // Purple has high red and blue, green has high green
        // Need to account for lighting conditions and sensor sensitivity

        // Simple color detection logic for purple vs green
        // Purple: high red and blue compared to green
        // Green: high green compared to red and blue
        if (red > green && blue > green && (red + blue) > (green * 1.5)) {
            return "PURPLE";
        } else if (green > red && green > blue) {
            return "GREEN";
        } else {
            // If no clear dominant color, return a default
            return "UNKNOWN";
        }
    }

    /**
     * Finds the appropriate slot based on the target pattern and detected color
     * @param targetPattern The ordered list of 3 colors to match
     * @param currentIndex The current index in the target pattern
     * @param detectedColor The color of the incoming ball
     * @return The slot number where the ball should be stored
     */
    private int findTargetSlot(String[] targetPattern, int currentIndex, String detectedColor) {
        // Look for the detected color in the remaining positions of the target pattern
        for (int i = currentIndex; i < targetPattern.length; i++) {
            if (targetPattern[i].equalsIgnoreCase(detectedColor)) {
                // Return the slot corresponding to this position in the pattern
                return i;
            }
        }

        // If the detected color isn't in the remaining pattern,
        // store it in the next available slot
        for (int i = 0; i < 3; i++) {
            if (!slotOccupied[i]) {
                return i;
            }
        }

        // If all slots are occupied, return -1 (shouldn't happen due to max 3 balls constraint)
        return -1;
    }

    /**
     * Resets the barrel controller state
     */
    public void reset() {
        currentBallIndex = 0;
        slotOccupied[0] = false;
        slotOccupied[1] = false;
        slotOccupied[2] = false;

        // Move to slot 0 as default position
        rotateToSlot(0);
    }

    /**
     * Checks if the barrel is full (all 3 slots occupied)
     * @return true if the barrel is full, false otherwise
     */
    public boolean isFull() {
        return currentBallIndex >= 3;
    }

    /**
     * Gets the current ball index (next position to store a ball)
     * @return The current ball index
     */
    public int getCurrentBallIndex() {
        return currentBallIndex;
    }

    /**
     * Checks if a specific slot is occupied
     * @param slotNumber The slot number to check
     * @return true if the slot is occupied, false otherwise
     */
    public boolean isSlotOccupied(int slotNumber) {
        if (slotNumber < 0 || slotNumber > 2) {
            return false;
        }
        return slotOccupied[slotNumber];
    }
}