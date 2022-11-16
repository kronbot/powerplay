package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        robotControl = new RobotControl(robot, telemetry);
        slideControl = new SlideControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
    }


    @Override
    public void loop() {
        boolean move = robotControl.rotate(gamepad1.right_stick_x);

        if (!move) {
            move |= robotControl.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            move |= robotControl.translate(gamepad1.left_trigger, gamepad1.right_trigger);
        }
        slideControl.slide(gamepad1.dpad_down, gamepad1.dpad_up);
        // slideControl.arm(gamepad1.y, gamepad1.a);
        // slideControl.intake(gamepad1.x, gamepad1.b);
        slideControl.slideLevels(gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x);

        // robot.slideDc.setTargetPosition(5000);
        // robot.slideDc.setPower(1);

        // robot.slideDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!move)
            robotControl.stop();
    }
}
