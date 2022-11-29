package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor test <3")
public class MotorTest extends OpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.y)
            robot.frontLeftDc.setPower(1.0);
        else
            robot.frontLeftDc.setPower(0);
        if(gamepad1.x)
            robot.frontRightDc.setPower(1.0);
        else
            robot.frontRightDc.setPower(0);
        if(gamepad1.b)
            robot.backLeftDc.setPower(1.0);
        else
            robot.backLeftDc.setPower(0);
        if(gamepad1.a)
            robot.backRightDc.setPower(1.0);
        else
            robot.backRightDc.setPower(0);
    }
}
