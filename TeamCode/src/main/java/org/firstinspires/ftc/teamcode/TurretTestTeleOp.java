package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.TurretManager;

@TeleOp
public class TurretTestTeleOp extends OpMode {
    TurretManager turret;

    @Override
    public void init() {
        turret = new TurretManager(hardwareMap);
    }

    @Override
    public void loop() {
        turret.update(telemetry);
    }
}
