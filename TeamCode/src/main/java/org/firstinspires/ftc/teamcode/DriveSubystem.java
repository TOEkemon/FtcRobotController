package org.firstinspires.ftc.teamcode;

import com.qualcom.robotcore.hardware.DcMotor;
import com.qualcom.robotcore.hardware.DcMotorEx;
import com.qualcom.robotcore.hardware.DcMotorSimple;
import com.qualcom.robotcore.hardware.HardwareMap;
import arcrobotics.ftclib.command.SubsystemBase;
import com.qualcom.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcom.robotcore.util.ElapsedTime;
import org.qualcom.robotcore.external.navigation.Pose2d;
import org.firstinspires.ftc.teamcode.DriveLocalizer;
import org.firstinspires.ftc.teamcode.Telemetry;
import org.firstinspires.ftc.teamcode.Rotation2d;
import org.firstinspires.ftc.teamcode.driveToCoord;





public class DriveSubystem extends SubsystemBase {

    DcMotorEx LF, LB, RF, RB;

    private MecanumDrive mecanumDrive;
    private IMU imu;
    public DriveLocalizer localizer;
    public Pose2d currentPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    ElapsedTime timer = new ElapsedTime();







    // auto
    public DriveSubystem(DcMotorEx LF, DcMotorEx LB, DcMotorEx RF, DcMotorEx RB, MecanumDrive mecanumDrive, Telemetr telemetry) {
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
    }   

    //teleop
    public DriveSubystem(DcMotorEx LF, DcMotorEx LB, DcMotorEx RF, DcMotorEx RB, MecanumDrive mecanumDrive, Telemetry telemetry) {
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx motor : new DcMotorEx[]{LF, LB, RB, RF}) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void driveFieldRelative(double left, double strafe, double right) {
        mecanumDrive.driveFieldRelative(left, strafe, right);
    }

    public void driveToCoord(double goalX, double goalY, double goalFieldHeading) {
        driveToCoord.driveToCoord(goalX, goalY, goalFieldHeading);
    }

    public static double getgoalFieldHeading(double distanceX, double distanceY) {
        double goalFieldHeading = 0;

        if (distanceX < 0) {
            goalFieldHeading = 180;
        } else if (distanceY < 0) {
            goalFieldHeading = 270;
        } else if (distanceY > 0) {
            goalFieldHeading = 90;
        } else {
            goalFieldHeading = 0;
        }

        return goalFieldHeading;
    }
    
}    
   




      



