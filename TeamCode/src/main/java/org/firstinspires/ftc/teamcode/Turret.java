import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;




public class Turret extends SubsystemBase {

    private DcMotorEx turretMotor;
    private double turretPosition;
    private double turretAngle;
    private double turretSpeed;
    private double turretMaxSpeed;
    private double turretMinSpeed;
    private double turretMaxAngle;
    private double turretMinAngle;
    private Telemetry telemetry;
    private double targetAngle;

    private static final double TICKS_PER_MOTOR_REV = 537.7;
    private static final double GEAR_RATIO = 5.0;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;
    private static final double kP = 0.02;          // power per degree of error, tune this
    private static final double ANGLE_TOLERANCE = 1.0;
    
    

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretPosition = 0.0;
        turretAngle = 0.0;
        turretSpeed = 0.0;
        turretMaxSpeed = 1.0;
        turretMinSpeed = -1.0;
        turretMaxAngle = 180.0;
        turretMinAngle = 0.0;

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void periodic() {
        turretAngle = getCurrentTurretAngle();
        turretPosition = turretMotor.getCurrentPosition();
        updateTurret();

        if (telemetry != null) {
            telemetry.addData("Turret Position", turretPosition);
            telemetry.addData("Turret Angle (raw)", turretAngle);
            telemetry.addData("Turret Speed", turretSpeed);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Turret Angle (wrapped)", turretAngle);
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    


    public void setTurretSpeed(double speed) {
        turretSpeed = Math.max(turretMinSpeed, Math.min(turretMaxSpeed, speed));
        turretMotor.setPower(turretSpeed);
    }


    public double getCurrentTurretAngle() {
        return turretMotor.getCurrentPosition()  / TICKS_PER_DEGREE;
        
    }

    public double getWrappedAngle() {
        return wrapAngle(getCurrentTurretAngle());
    }

    public double wrapAngle(double angle) {
        return ((angle + 180.0) % 360.0 + 360.0) % 360 - 180;
    }


    public void setTurretAngle(double angle) {
        turretAngle = Math.max(turretMinAngle, Math.min(turretMaxAngle, angle));
        
        turretPosition = (turretAngle - turretMinAngle) / (turretMaxAngle - turretMinAngle);

        double current = getCurrentTurretAngle();
        double delta = (angle - current);
        double candidate = current + delta;

    if (candidate > turretMaxAngle) {
        candidate -= 360;
    }    

    if (candidate < turretMinAngle) {
        candidate  += 360;
    }

    targetAngle = Math.max(turretMinAngle, Math.min(turretMaxAngle, candidate));
    }

    public void updateTurret() {
        double error = targetAngle - getCurrentTurretAngle();
        double power =  Math.abs(error) < ANGLE_TOLERANCE? 0.0 : kP * error;
        setTurretSpeed(power);
    }

    public boolean atTarget() {
        return Math.abs(targetAngle - getCurrentTurretAngle()) < ANGLE_TOLERANCE;
    }


    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetAngle = 0.0;

    }

    public void stopTurret() {
        turretMotor.setPower(0.0);
    }

} 
