
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

  public class MecanumDrive {

      public static class Params {
          public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                  RevHubOrientationOnRobot.LogoFacingDirection.UP;
          public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                  RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

          // Drive model (tune these!)
          public double inPerTick = 1;          // ForwardPushTest
          public double lateralInPerTick = 1;   // LateralPushTest / LateralRampLogger
          public double trackWidthTicks = 0;    // AngularRampLogger  (must be > 0!)

          // Feedforward, in TICK units like the quickstart (ForwardRampLogger / ManualFeedforwardTuner)
          public double kS = 0;
          public double kV = 0;
          public double kA = 0;
      }

      public static Params PARAMS = new Params();

      public MecanumKinematics kinematics;
      public MotorFeedforward feedforward;
      public DriveLocalizer localizer;

      public DcMotorEx leftFront, leftBack, rightBack, rightFront;
      public IMU imu;
      public VoltageSensor voltageSensor;

      public void init(HardwareMap hwMap, Pose2d startPose) {
          leftFront  = hwMap.get(DcMotorEx.class, "FL");
          leftBack   = hwMap.get(DcMotorEx.class, "BL");
          rightBack  = hwMap.get(DcMotorEx.class, "BR");
          rightFront = hwMap.get(DcMotorEx.class, "FR");

          // Typical mecanum: reverse the LEFT side. Verify on your robot.
          leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
          leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

          for (DcMotorEx m : new DcMotorEx[]{leftFront, leftBack, rightBack, rightFront}) {
              m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // feedforward does the velocity work
          }

          imu = hwMap.get(IMU.class, "imu");
          imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                  PARAMS.logoFacingDirection, PARAMS.usbFacingDirection)));
          imu.resetYaw();

          voltageSensor = hwMap.voltageSensor.iterator().next();

          // Built here (not as field initializers) so they use the current PARAMS values
          kinematics = new MecanumKinematics(
                  PARAMS.inPerTick * PARAMS.trackWidthTicks,
                  PARAMS.inPerTick / PARAMS.lateralInPerTick);

          feedforward = new MotorFeedforward(
                  PARAMS.kS,
                  PARAMS.kV / PARAMS.inPerTick,
                  PARAMS.kA / PARAMS.inPerTick);

          localizer = new DriveLocalizer(startPose);
      }

      /** TeleOp: normalized powers (-1..1). x = forward, y = LEFT, heading = CCW. */
      public void setDrivePowers(PoseVelocity2d powers) {
          // Track width 1 => same math as your manual mixing, just done by RR
          MecanumKinematics.WheelVelocities<Time> w =
                  new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

          double max = 1;
          for (DualNum<Time> p : w.all()) max = Math.max(max, Math.abs(p.value()));

          leftFront.setPower(w.leftFront.value() / max);
          leftBack.setPower(w.leftBack.value() / max);
          rightBack.setPower(w.rightBack.value() / max);
          rightFront.setPower(w.rightFront.value() / max);
      }

      /** Field-centric TeleOp. Call localizer.update() every loop first. */
      public void driveFieldRelative(double forward, double left, double turn) {
          Rotation2d heading = localizer.getPose().heading;
          Vector2d robotVec = heading.inverse().times(new Vector2d(forward, left));
          setDrivePowers(new PoseVelocity2d(robotVec, turn));
      }

      /** Real units (in/s, rad/s) -> kinematics -> feedforward volts -> power. */
      public void setDriveVelocity(PoseVelocity2d vel) {
          MecanumKinematics.WheelVelocities<Time> w =
                  kinematics.inverse(PoseVelocity2dDual.constant(vel, 2)); // 2 = vel + accel(0)

          double voltage = voltageSensor.getVoltage();
          leftFront.setPower(feedforward.compute(w.leftFront) / voltage);
          leftBack.setPower(feedforward.compute(w.leftBack) / voltage);
          rightBack.setPower(feedforward.compute(w.rightBack) / voltage);
          rightFront.setPower(feedforward.compute(w.rightFront) / voltage);
      }

      public void stop() {
          setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
      }

      // ---------------- Drive-encoder + IMU localizer ----------------
      public class DriveLocalizer {
          public final Encoder leftFront, leftBack, rightBack, rightFront;

          private int lastLF, lastLB, lastRB, lastRF;
          private Rotation2d lastHeading;
          private boolean initialized = false;
          private Pose2d pose;

          public DriveLocalizer(Pose2d startPose) {
              leftFront  = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
              leftBack   = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
              rightBack  = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
              rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

              // RawEncoder ignores the motor's direction, so you'll likely need these
              // for the motors you reversed. Check: push robot forward -> x must increase.
              leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
              leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

              pose = startPose;
          }

          public Pose2d getPose() { return pose; }
          public void setPose(Pose2d p) { pose = p; }

          /** Call once per loop. Returns robot velocity. */
          public PoseVelocity2d update() {
              PositionVelocityPair lf = leftFront.getPositionAndVelocity();
              PositionVelocityPair lb = leftBack.getPositionAndVelocity();
              PositionVelocityPair rb = rightBack.getPositionAndVelocity();
              PositionVelocityPair rf = rightFront.getPositionAndVelocity();

              Rotation2d heading = Rotation2d.exp(
                      imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

              if (!initialized) {
                  initialized = true;
                  lastLF = lf.position; lastLB = lb.position;
                  lastRB = rb.position; lastRF = rf.position;
                  lastHeading = heading;
                  return new PoseVelocity2d(new Vector2d(0, 0), 0);
              }

              double headingDelta = heading.minus(lastHeading);

              Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                      new DualNum<Time>(new double[]{lf.position - lastLF, lf.velocity}).times(PARAMS.inPerTick),
                      new DualNum<Time>(new double[]{lb.position - lastLB, lb.velocity}).times(PARAMS.inPerTick),
                      new DualNum<Time>(new double[]{rb.position - lastRB, rb.velocity}).times(PARAMS.inPerTick),
                      new DualNum<Time>(new double[]{rf.position - lastRF, rf.velocity}).times(PARAMS.inPerTick)
              ));

              lastLF = lf.position; lastLB = lb.position;
              lastRB = rb.position; lastRF = rf.position;
              lastHeading = heading;

              // Wheels give x/y; IMU gives heading (much more accurate than wheel heading)
              pose = pose.plus(new Twist2d(twist.line.value(), headingDelta));

              return twist.velocity().value();
          }
      }
  }












 */
