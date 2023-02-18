package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.SlideControl;

@TeleOp (name = "Dual Manual trigonometry", group = "Dual")
public class DualManualTrig extends OpMode {
    private KronBot robot;
    private RobotControl robotControl;
    private SlideControl slideControl;

    public DualManualTrig() {
        robot = new KronBot();
        robotControl = new RobotControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
    }

    @Override
    public void loop() {

        boolean move = robotControl.rotate(gamepad1.right_stick_x/1.25);
        if (!move)
            move = robotControl.translate(gamepad1.right_trigger/2, gamepad1.left_trigger/2);
        if (!move)
            move = robotControl.drive(0, -gamepad1.left_stick_y);
        if (!move)
            robotControl.stop();
//        robotControl.debug();

        slideControl.intake(gamepad2.dpad_up);
        slideControl.control(gamepad2, false);
    }
}

