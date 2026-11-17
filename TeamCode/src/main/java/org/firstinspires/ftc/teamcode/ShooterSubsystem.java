package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.command.SubystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.naviagation.Pose2D;
import org.firstinpsires.ftc.teamcode.utilities.ShooterConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;





public class ShooterSubsystem extends SubsystemBase  {

    private Servo hoodServo;;
    private DcMotorEx shooterMotor;
    private PIDFController velocityPIDfController;
    public static double HOOD_SERVO_UP_POSITION = 0.0;
    public static double HOOD_SERVO_DOWN_POSITION = 0.0;

    private Pose2D currentPose;
    private AngleUnit angleUnit;
    private double targetX, targetY, targetAngle;
    private double target_Position;




    public void Shooter(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.direction.REVERSE);



        velocityPIDfController = new PIDFController(
            ShooterConstants.kP,
            ShooterConstants.kI, 
            ShooterConstants.kD, 
            ShooterConstants.kF);
        
        velocityPIDFController.setTolerance(100);
        
    }
    public void setVelocityPowerPoint() {
        
    }
    

    public void onInit() {
    shooterMotor.setPower(0.0);
    hoodServo.setPosition();




    }

    public void setHoodServoPosition(double position, AngleUnit angleUnit) {

        hoodServos.setHoodServoPosition(position);
        HoodToAngleUnit(angleUnit);


    }


    public void setVelocityPoint(double target_Position) {
    }

    public double get HoodServoPosition() {
       return hoodServo.getPosition();

    }

    public double getCurrentShooterPower(double) {
