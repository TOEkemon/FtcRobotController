package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretManager {
    private final DcMotorEx flywheel;
    public final double EXIT_ANGLE = 75;
    public final double EXIT_HEIGHT = 12;

    public TurretManager(HardwareMap hardwareMap) {
        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel");
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("Flywheel RPM:", flywheel.getVelocity(AngleUnit.DEGREES) / 6);
    }
}
