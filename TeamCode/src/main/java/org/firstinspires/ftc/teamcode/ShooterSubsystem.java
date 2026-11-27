
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utilities.InterpLUT;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ShooterSubsystem extends SubsystemBase {

    private DcMotorEx flyWheelMotor;
    private PIDController velocityPIDController;
    private Telemetry telemetry;
    private VoltageSensor battery;
    private final InterpLUT shooterInteroperableMap;

    private static final double kP = 0.005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.05;
    private static final double kV = 0.00040; 
    private static final double VELOCITY_TOLERANCE = 40;
    private static final double AIM_TOLERNACE_DEGREE = 2.0;
    private static final double SHOOTER_HEADING_OFFSET_DEG = 0;

    private static final double MIN_DIST_IN = 24;
    private static final double MAX_DIST_IN = 120;
    private static final double NOMINAL_VOLTAGE = 12.0;
    

    private Pose2D currentPose;
    private double targetX, targetY;
    private boolean isFlyWheelEnabled;
    private double manualVelocity = -1;
    private double distanceToTarget = 0;
    private double targetVelocity = 0;
    private double headingErrorRad = 0;
    private boolean enabled = false;
    private double targetPosition;

    

    public ShooterSubsystem(HardwareMap hwMap) {
        flyWheelMotor = hwMap.get(DcMotorEx.class, "shooterMotor");
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        velocityPIDController = new PIDController(kP, kI, kD);
        velocityPIDController.setTolerance(100);

        shooterInteroperableMap = new InterpLUT()
            .add(24, 1100.0)
            .add(36, 1200.0)
            .add(48, 1300.0)
            .add(60.0, 1420.0)
            .add(72.0, 1530.0)
            .add(84.0, 1650.0)
            .add(96.0, 1760.0)
            .add(120.0, 1980.0)
            .createLUT();
    }

    public void onInit() {
        enabled = false;
        flyWheelMotor.setPower(0.0);
    }

    public void setFlyWheelEnabled(boolean on) {
        enabled = on;
        if (!on) {
            flyWheelMotor.setPower(0.0);
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;

    }
    public void setCurrentPose(Pose2D pose) {
        currentPose = pose;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public double getCurrentVelocity() {
        return flyWheelMotor.getVelocity();
    }

    public void setTarget( double x, double y) {
        targetX = x;
        targetY = y;
    } 
    public void setManualVelocity(double v) {
        manualVelocity = v;
    }

    public boolean isAimed() {
         return Math.abs(headingErrorRad) < Math.toRadians(AIM_TOLERNACE_DEGREE);
    }
    
    
    public boolean isAtSpeed() {
        return enabled && targetVelocity > 0
         && Math.abs(targetVelocity - getCurrentVelocity()) < VELOCITY_TOLERANCE;
    }

    public boolean readyToShoot() {
        return isAtSpeed() && isAimed();
    }

    @Override
    public void periodic() {
        velocityPIDController.setPID(kP, kI, kD);

        updateAim();

        if (manualVelocity >= 0) {
            targetVelocity = manualVelocity;
        } else if (currentPose != null) {
            double d = Range.clip(distanceToTarget, MIN_DIST_IN, MAX_DIST_IN);
            targetVelocity = shooterInteroperableMap.get(d);
        } else { targetVelocity = 0;}
        double currentVelocity = getCurrentVelocity();
        double power = 0;
        
        if (enabled && targetVelocity >0) {
            double ff = kV * kS * targetVelocity;
            ff *= NOMINAL_VOLTAGE/ battery.getVoltage();
            double fb = velocityPIDController.calculate( currentVelocity, targetVelocity);
            power = Range.clip(ff + fb, 0.0 , 1.0);

        } else {
            velocityPIDController.reset();
        }
        flyWheelMotor.setPower(power);

       if (telemetry !=null) {
        telemetry.addData("distance to hive", distanceToTarget);
        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("VelocitySetpoint", targetPosition);
        telemetry.addData(" isAimed/isAtSpeed", isAimed() + "" + isAtSpeed());
       } 
    }
    


     
    public void setVelocitySetPoint(double targetPosition) {
        if (!isFlyWheelEnabled) {
            return;
        }
        
        velocityPIDController.setSetPoint(targetPosition);
        velocityPIDController.setPID(kP, kI, kD);
        flyWheelMotor.setPower(
                velocityPIDController.calculate(flyWheelMotor.getVelocity(), targetPosition)
        );
      

    }
    

    public double calculateFlyWheelSpeeds(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;

        if (currentPose == null) {
            return 0.0;
        }

        double poseX = currentPose.getX(DistanceUnit.INCH);
        double poseY = currentPose.getY(DistanceUnit.INCH);
        double distanceToTarget = Math.hypot(targetX - poseX, targetY - poseY);

        return shooterInteroperableMap.get(distanceToTarget);
    }
    
    public void updateAim() {
        if (currentPose ==null) { return; }
        double dx = targetX - currentPose.getX(DistanceUnit.INCH);
        double dy = targetY - currentPose.getY(DistanceUnit.INCH);
        distanceToTarget = Math.hypot(dy, dx);

        double angleToTarget = Math.atan2(dy, dx);
        double robotHeading =  currentPose.getHeading(AngleUnit.RADIANS)
        + Math.toRadians(SHOOTER_HEADING_OFFSET_DEG);
        headingErrorRad = AngleUnit.normalizeRadians(angleToTarget - robotHeading);
    }

    
}
