package org.firstinspires.ftc.teamcode.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.SlideControl;

@TeleOp (name = "Dual Manual Control", group = "Dual")
public class DualManualControl extends OpMode {
    private KronBot robot;
    private RobotControl robotControl;
    private SlideControl slideControl;

    public DualManualControl() {
        robot = new KronBot();
        robotControl = new RobotControl(robot, telemetry);
        slideControl = new SlideControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.resetSlideEncoder();
        robot.controlIntake(1);
    }

    @Override
    public void loop() {
        boolean move = robotControl.rotate(gamepad1.right_stick_x/1.25);
        if (!move)
            move = robotControl.translate(gamepad1.right_trigger, gamepad1.left_trigger);
        if (!move)
            move = robotControl.drive(0, -gamepad1.left_stick_y);
        if (!move)
            robotControl.stop();
//        robotControl.debug();

        slideControl.intake(gamepad2.dpad_up);
        slideControl.control(gamepad2, false);
    }
}