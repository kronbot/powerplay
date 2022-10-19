package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotControl;

@TeleOp(name = "Manual Control")
public class ManualControl extends OpMode {
    private final KronBot robot;
    private final RobotControl control;

    public ManualControl() {
        robot = new KronBot();
        control = new RobotControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
    }


    @Override
    public void loop() {
        boolean move = false;
        move |= control.rotate(gamepad1.right_stick_x);
        move |= control.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        move |= control.translate(gamepad1.left_trigger, gamepad1.right_trigger);
        control.slide(gamepad1.dpad_down, gamepad1.dpad_up);

        if (!move)
            robot.stopMotors();
    }
}
