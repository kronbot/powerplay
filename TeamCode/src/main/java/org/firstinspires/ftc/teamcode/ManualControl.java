package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotControl;
import org.firstinspires.ftc.teamcode.utils.SlideControl;
import org.firstinspires.ftc.teamcode.utils.SlideLevelControl;

@TeleOp(name = "Single Manual Control")
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

        if (!move)
            move |= robotControl.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);

        telemetry.addData("Move state", move);
        if (!move)
            robotControl.stop();

        slideLevelControl.loop(
                gamepad1.x,
                gamepad1.y,
                gamepad1.b,
                gamepad1.a,
                false
        );

        // slideControl.slide(
        //     gamepad1.right_trigger > 0 ? true : false,
        //     gamepad1.left_trigger > 0 ? true : false
        // );
        slideControl.arm(gamepad1.dpad_down, gamepad1.dpad_up);
        slideControl.intake(gamepad1.dpad_right, gamepad1.dpad_left);
    }
}
