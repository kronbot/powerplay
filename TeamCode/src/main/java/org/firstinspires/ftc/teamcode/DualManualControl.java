package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotControl;
import org.firstinspires.ftc.teamcode.utils.SlideControl;
import org.firstinspires.ftc.teamcode.utils.SlideLevelControl;

@TeleOp(name = "Dual Manual Control")
public class DualManualControl extends OpMode {
    private final KronBot robot;
    private final RobotControl robotControl;
    private final SlideControl slideControl;
    private final SlideLevelControl slideLevelControl;

    public DualManualControl() {
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

        slideLevelControl.loop(
                gamepad2.x,
                gamepad2.y,
                gamepad2.b,
                gamepad2.a,
                true
        );

        slideControl.slide(
                gamepad2.right_trigger > 0 ? true : false,
                gamepad2.left_trigger > 0 ? true : false
        );
        slideControl.arm(gamepad2.dpad_down, gamepad2.dpad_up);
        slideControl.intake(gamepad2.dpad_right, gamepad2.dpad_left);
    }
}
