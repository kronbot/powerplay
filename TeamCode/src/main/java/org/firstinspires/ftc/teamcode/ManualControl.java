package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manual Control")
public class ManualControl extends OpMode {
    KronBot robot = new KronBot();

    @Override
    public void init() {
        robot.initHardwareMap();
    }

    void driveWheels() {

    }

    @Override
    public void loop() {
        driveWheels();
    }
}
