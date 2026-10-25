package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.navigation.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.naviagation.Pose2D;





@Autonomous
(name = "AutoTurret", group ="automonous")
public class AutoTurretShooter {

    private Servo gateServo, hoodServo;;
    private DcMotor shooterMotor;

    public static final double  GATE_SERVO_CLOSED_POSITION = 0.0;
    public static final double GATE_SERVO_OPEN_POSITION = 0.0;
    public static final double HOOD_SERVO_UP_POSITION = 0.0;
    public static final double HOOD_SERVO_DOWN_POSITION = 0.0;

    private Pose2D currentPose;
    private AngleUnit angleUnit;
    private double targetX, targetY, targetAngle;




    public AutoTurret(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        // Set initial positions for servos
        gateServo.setPosition(GATE_SERVO_CLOSED_POSITION);
        hoodServo.setPosition(HOOD_SERVO_UP_POSITION);

    
    }

   

    public void AlignToTarget(Pose2D targetPose, AngleUnit angleUnit) {

        // Implementation for driving to target
        Posed2D currentPose = getCurrentPose();

    


        double targetAngle = CalculateTargetAngle(targetX, targetY);
        

    }

    public void setGateServoPosition(double position, AngleUnit angleUnit) {
        gateServo.setPosition(position);
        gateToAngleUnit(angleUnit);


        
    }

    public void setHoodServoPosition(double position, AngleUnit angleUnit) {

        hoodServos.setHoodServoPosition(position);
        HoodToAngleUnit(angleUnit);
     
       
    }


}
