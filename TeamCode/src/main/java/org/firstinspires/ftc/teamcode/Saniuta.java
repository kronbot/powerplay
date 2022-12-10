package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@TeleOp(name = "Saniuta")
public class Saniuta extends OpMode {
    private KronBot robot = new KronBot();
    private boolean front = true;

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up)
            front = !front;
        if (gamepad1.x)
            robot.frontLeftDc.setPower(front ? 1 : -1);
        else
            robot.frontLeftDc.setPower(0);
        if (gamepad1.y)
            robot.frontRightDc.setPower(front ? 1 : -1);
        else
            robot.frontRightDc.setPower(0);
        if (gamepad1.b)
            robot.backRightDc.setPower(front ? 1 : -1);
        else
            robot.backRightDc.setPower(0);
        if (gamepad1.a)
            robot.backLeftDc.setPower(front ? 1 : -1);
        else
            robot.backLeftDc.setPower(0);
    }
}