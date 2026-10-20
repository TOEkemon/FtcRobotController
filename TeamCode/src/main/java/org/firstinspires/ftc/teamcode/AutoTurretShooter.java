ftc.package.org.firstinspires.ftc.teamcode.Subsystems;
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import spark.org.r
i



@Autonomous
(name = "TurretShooter", group = "Subsystems")
public class AutoTurretShooter {

    private Servo gateServo, hoodServo;;
    private DcMotor shooterMotor;

    public static final double GATE_SERVO_OPEN_POSITION = 0.0;
    public static final double GATE_SERVO_CLOSED_POSITION = 0.0;
    public static final double HOOD_SERVO_UP_POSITION = 0.0;
    public static final double HOOD_SERVO_DOWN_POSITION = 0.0;

    

public enum SHOOTER_STATE {
        IDLE,
        SHOOTING,
        RELOADING
    }

    private SHOOTER_STATE currentState;

    public AutoTurretShooter(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        // Set initial positions for servos
        gateServo.setPosition(GATE_SERVO_CLOSED_POSITION);
        hoodServo.setPosition(HOOD_SERVO_UP_POSITION);

        // Set initial state
        currentState = SHOOTER_STATE.IDLE;
    }

    public void setShooterState(SHOOTER_STATE newState) {
        currentState = newState;
    }

    public SHOOTER_STATE getShooterState() {
        return currentState;
    }
}