public class ShooterConstants {
    public static final String kSubsystemName = "Shooter ";
    public static final String shooterMotorID = "shooterMotor";
    public static final String hoodServoID = "hoodServo";
    public static final String blockerServoID = "blockServo";

    public static final double kShooterIdleRPM = 1500;
    public static final double kShooterMaximumAchievableRPM = 5600;

    public static final double kHoodMinimumPosition = 1.0;
    public static final double kHoodMaximumPosition = 0.0;

    public static double kTuningFlywheelVelocitySetpoint = 0;
    public static double kTuningHoodPositionSetpoint = kHoodMinimumPosition;

    public static final double shooterMotorCPR = 28;

    public static double kP = 0.0015;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double kS = 0.031;
    public static double kV = 0.000165;
    public static double kA = 0.000001 * 12;
}
