package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotControl;
import org.firstinspires.ftc.teamcode.utils.SlideControl;

@TeleOp(name = "Manual Control")
public class ManualControl extends OpMode {
    private final KronBot robot;
    private final RobotControl robotControl;
    private final SlideControl slideControl;

    public ManualControl() {
        robot = new KronBot();
        robotControl = new RobotControl(robot, telemetry, hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.resetSlideEncoder();
    }

    @Override
    public void loop() {
        boolean move = robotControl.rotate((gamepad1.right_bumper ? 1 : 0) - (gamepad1.left_bumper ? 1 : 0));
        if (!move)
            move = robotControl.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        if (!move)
            robotControl.stop();
        robotControl.debug();

        slideControl.slide(gamepad1.left_trigger > 0, gamepad1.right_trigger > 0);
        slideControl.intake(gamepad1.dpad_right, gamepad1.dpad_left);
        // checking if the current state is finished
//        slideLevelControl.loop(
//                gamepad1.dpad_right,
//                gamepad1.dpad_left,
//                gamepad1.dpad_up,
//                gamepad1.dpad_down,
//                true
//        );
    }
}
