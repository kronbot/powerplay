package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Slide test pt stefan")
public class SlideTest extends OpMode {
    private KronBot robot = new KronBot();

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up)
            robot.slideDc.setPower(0.4);
        else if (gamepad1.dpad_down)
            robot.slideDc.setPower(-0.2);
        else
            robot.slideDc.setPower(0.1);
    }
}