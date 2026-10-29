package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.VisionManager;

@TeleOp
public class VisionTestTeleOp extends OpMode {
    VisionManager vision;

    @Override
    public void init() {
        vision = new VisionManager(hardwareMap);
    }

    @Override
    public void loop() {
        vision.update(telemetry);
    }

    @Override
    public void stop() {
        vision.close();
    }
}