package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.SlideControl;

@TeleOp (name = "Manual Control with Direction Control", group = "Single")
public class ManualControlDirrectionControl extends OpMode {
    private final KronBot robot;
    private final RobotControl robotControl;
    private SlideControl slideControl;

    public ManualControlDirrectionControl() {
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
        boolean move = robotControl.rotate(gamepad1.left_stick_x);
        if (!move)
            move = robotControl.translate(
                    gamepad1.right_stick_x < 0 ? gamepad1.right_stick_x : 0,
                    gamepad1.right_stick_x > 0 ? -gamepad1.right_stick_x : 0
            );
        if (!move)
            move = robotControl.drive(gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
        if (!move)
            robotControl.stop();
//        robotControl.debug();

        slideControl.intake(gamepad1.dpad_up);
        slideControl.control(gamepad1, false);
    }
}
