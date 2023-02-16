package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.autonomous.configurations.TestConfiguration;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.autonomous.GlobalCoordinatePosition;

@TeleOp(name = "Manual Control with encoders", group = "Single")
public class ManualControlWithEncoders extends OpMode {
    private final KronBot robot = new KronBot();
    private RobotControl robotControl;
    private GlobalCoordinatePosition position;
    private Thread thread;

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robotControl = new RobotControl(robot, telemetry);
        position = new GlobalCoordinatePosition(
                robot.leftEncoder,
                robot.rightEncoder,
                robot.frontEncoder,
                new TestConfiguration(),
                telemetry
        );
        thread = new Thread(position);
        thread.start();
    }

    @Override
    public void loop() {
        boolean move = robotControl.rotate(gamepad1.right_stick_x);
        if (!move) {
            double x = Math.abs(gamepad1.left_stick_x) < 0.25 ? gamepad1.left_stick_x : (gamepad1.left_stick_x > 0 ? 1 : -1) * 0.25;
            double y = Math.abs(gamepad1.left_stick_y) < 0.25 ? gamepad1.left_stick_y : (gamepad1.left_stick_y > 0 ? 1 : -1) * 0.25;

            move = robotControl.drive(-x, -y);
        }
        if (!move)
            robotControl.stop();
    }

    @Override
    public void stop() {
        position.stop();
        super.stop();
    }
}
