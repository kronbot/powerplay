package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotControl;
import org.firstinspires.ftc.teamcode.utils.SlideControl;
import org.firstinspires.ftc.teamcode.utils.SlideLevelControl;

@TeleOp(name = "Manual Control")
public class ManualControl extends OpMode {
    private final KronBot robot;
    private final RobotControl robotControl;
    private final SlideControl slideControl;
    private final SlideLevelControl slideLevelControl;

    public ManualControl() {
        robot = new KronBot();
        robotControl = new RobotControl(robot, telemetry);
        slideControl = new SlideControl(robot, telemetry);
        slideLevelControl = new SlideLevelControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.resetSlideEncoder();
    }


    @Override
    public void loop() {
        boolean move = robotControl.rotate(gamepad1.right_stick_x);
        if (!move) {
            int rotateDirection = (gamepad1.right_bumper ? 1 : 0) - (gamepad1.left_bumper ? 1 : 0);
            move = robotControl.rotate(rotateDirection);
        }

        if (!move) {
            move |= robotControl.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            move |= robotControl.translate(gamepad1.left_trigger, gamepad1.right_trigger);
        }

        telemetry.addData("Move state", move);
        if (!move)
            robotControl.stop();

        // checking if the current state is finished
        slideLevelControl.loop(
                gamepad1.dpad_right,
                gamepad1.dpad_left,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                true
        );
    }
}
