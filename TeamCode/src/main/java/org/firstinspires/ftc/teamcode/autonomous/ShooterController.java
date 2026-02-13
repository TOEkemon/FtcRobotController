package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ShooterController handles the shooter motor and ball pushing mechanism.
 * It manages the shooting sequence and timing for launching balls.
 */
public class ShooterController {
    
    // Hardware components
    private DcMotor shooterMotor;
    private DcMotor kickerMotor;
    private Servo grip_servo_left;  // Left gripper servo
    private Servo grip_servo_right; // Right gripper servo
    
    // Constants for shooting timing
    private static final double BALL_PUSH_DURATION = 0.5; // seconds to push ball
    private static final double SHOT_DELAY = 0.3;         // seconds between shots
    
    // State tracking
    private ElapsedTime timer = new ElapsedTime();
    private boolean isShootingSequenceActive = false;
    private int currentShotIndex = 0;
    private boolean isBallPushing = false;
    
    /**
     * Constructor for ShooterController
     * @param shooterMotor The motor that spins the shooter wheel
     * @param kickerMotor The servo that pushes balls into the shooter
     * @param grip_servo_left Left gripper servo
     * @param grip_servo_right Right gripper servo
     */
    public ShooterController(DcMotor shooterMotor, DcMotor kickerMotor, Servo grip_servo_left, Servo grip_servo_right) {
        this.shooterMotor = shooterMotor;
        this.kickerMotor = kickerMotor;
        this.grip_servo_left = grip_servo_left;
        this.grip_servo_right = grip_servo_right;

        // Initialize servos to default positions if not null
        if (kickerMotor != null) {
            kickerMotor.setPower(0.0); // Retracted position
        }
        if (grip_servo_left != null) {
            grip_servo_left.setPosition(0.5); // Neutral position
        }
        if (grip_servo_right != null) {
            grip_servo_right.setPosition(0.5); // Neutral position
        }
    }
    
    /**
     * Starts the shooter motor and moves gripper servos to forward shooting position
     */
    public void startShooter() {
        // Set the shooter motor to run at full power
        shooterMotor.setPower(1.0);
        
        // Move gripper servos to forward shooting position (left to 1.0, right to 0.0)
        if (grip_servo_left != null) {
            grip_servo_left.setPosition(1.0);
        }
        if (grip_servo_right != null) {
            grip_servo_right.setPosition(0.0);
        }
    }
    
    /**
     * Starts the shooter motor in reverse and moves gripper servos to backward shooting position
     */
    public void startShooterReverse() {
        // Set the shooter motor to run in reverse at full power
        shooterMotor.setPower(-1.0);
        
        // Move gripper servos to backward shooting position (left to 0.0, right to 1.0)
        if (grip_servo_left != null) {
            grip_servo_left.setPosition(0.0);
        }
        if (grip_servo_right != null) {
            grip_servo_right.setPosition(1.0);
        }
    }

    /**
     * Stops the shooter motor and moves gripper servos to neutral position
     */
    public void stopShooter() {
        shooterMotor.setPower(0.0);
        
        // Move gripper servos to neutral position when stopping shooter
        if (grip_servo_left != null) {
            grip_servo_left.setPosition(0.5);
        }
        if (grip_servo_right != null) {
            grip_servo_right.setPosition(0.5);
        }
    }
    
    /**
     * Begins the shooting sequence for all stored balls
     */
    public void beginShootingSequence() {
        isShootingSequenceActive = true;
        currentShotIndex = 0;
        timer.reset();
    }
    
    /**
     * Updates the shooting sequence state
     * This should be called repeatedly during the shooting phase
     * @return true if the shooting sequence is still active, false if complete
     */
    public boolean updateShootingSequence() {
        if (!isShootingSequenceActive) {
            return false;
        }

        // If we've shot all 3 balls, end the sequence
        if (currentShotIndex >= 3) {
            isShootingSequenceActive = false;
            return false;
        }

        if (!isBallPushing) {
            // Move the ball push servo to push the ball if it's available
            if (kickerMotor != null) {
                kickerMotor.setPower(1.0); // Extended position to push ball
            }
            isBallPushing = true;
            timer.reset();
        } else if (timer.seconds() >= BALL_PUSH_DURATION) {
            // Retract the ball push servo after the push duration if it's available
            if (kickerMotor != null) {
                kickerMotor.setPower(0.0); // Retracted position
            }
            isBallPushing = false;

            // Wait for the shot delay before moving to the next ball
            timer.reset();
            while (timer.seconds() < SHOT_DELAY) {
                // Busy wait for the shot delay - in real implementation,
                // this would be handled differently to allow other operations
                // For now, we'll just return true to continue processing
                return true;
            }

            // Move to the next shot
            currentShotIndex++;

            // If we still have more balls to shoot, return true to continue
            if (currentShotIndex < 3) {
                return true;
            } else {
                isShootingSequenceActive = false;
                return false;
            }
        }

        return true; // Shooting sequence is still active
    }
    
    /**
     * Manually push a single ball
     */
    public void pushSingleBall() {
        // Push the ball if servo is available
        if (kickerMotor != null) {
            kickerMotor.setPower(1.0);
        }

        // Wait for the push duration
        ElapsedTime pushTimer = new ElapsedTime();
        pushTimer.reset();
        while (pushTimer.seconds() < BALL_PUSH_DURATION) {
            // Busy wait - in real implementation, this would be handled differently
        }

        // Retract the servo if available
        if (kickerMotor != null) {
            kickerMotor.setPower(0.0);
        }
    }
    
    /**
     * Checks if the shooting sequence is currently active
     * @return true if shooting sequence is active, false otherwise
     */
    public boolean isShootingSequenceActive() {
        return isShootingSequenceActive;
    }
    
    /**
     * Gets the current shot index (0-2)
     * @return The current shot index
     */
    public int getCurrentShotIndex() {
        return currentShotIndex;
    }
    
    /**
     * Resets the shooter controller state
     */
    public void reset() {
        stopShooter();
        // Retract the servo if available
        if (kickerMotor != null) {
            kickerMotor.setPower(0.0); // Retracted position
        }
        isShootingSequenceActive = false;
        currentShotIndex = 0;
        isBallPushing = false;
    }
    
    /**
     * Sets the shooter motor power
     * @param power Power level (-1.0 to 1.0)
     */
    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }
    
    /**
     * Gets the current shooter motor power
     * @return Current power level
     */
    public double getShooterPower() {
        return shooterMotor.getPower();
    }
}