package org.firstinspires.ftc.teamcode.manual;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.Utils;

@Config
@TeleOp (name = "Dual Manual Control", group = "Dual")
public class DualManualControl extends OpMode {
    public static double MAX_SPEED = 0.7;
    public static double MAX_TSPEED = 0.8;
    private final KronBot robot;
    private final RobotControl robotControl;
    private SlideControl slideControl;

    public DualManualControl() {
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
        double rotateInput = Utils.clamp(gamepad1.right_stick_x / 1.25, -MAX_SPEED, MAX_SPEED);
        boolean move = robotControl.rotate(gamepad1.right_stick_x/1.25);
        if (!move) {
            double leftInput = Utils.clamp(gamepad1.right_trigger / 2, -MAX_TSPEED, MAX_TSPEED);
            double rightInput = Utils.clamp(gamepad1.left_trigger / 2, -MAX_TSPEED, MAX_TSPEED);
            move = robotControl.translate(leftInput, rightInput);
        }
        if (!move) {
            move = robotControl.drive(0, Utils.clamp(-gamepad1.left_stick_y, -MAX_SPEED, MAX_SPEED));
        }
        if (!move)
            robotControl.stop();
//        robotControl.debug();

        slideControl.intake(gamepad2.dpad_up);
        slideControl.control(gamepad2, false);
    }
}
