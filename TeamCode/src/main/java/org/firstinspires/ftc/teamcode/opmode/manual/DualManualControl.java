package org.firstinspires.ftc.teamcode.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.ArmControl;
import org.firstinspires.ftc.teamcode.lib.RobotControl;

@TeleOp(name = "Dual Manual Control", group = "Manual")
public class DualManualControl extends OpMode {
    private final KronBot robot;
    private final RobotControl robotControl;
    private final ArmControl armControl;

    public DualManualControl() {
        robot = new KronBot();
        robotControl = new RobotControl(robot, telemetry);
        armControl = new ArmControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.resetArmEncoder();

        robot.clawServo.setPosition(1);
    }

    @Override
    public void loop() {
        boolean move = robotControl.rotate(gamepad1.right_stick_x   );
        if (!move)
            move = robotControl.translate(gamepad1.right_trigger, gamepad1.left_trigger);
        if (!move)
            move = robotControl.drive(0, -gamepad1.left_stick_y);
        if (!move)
            robotControl.stop();

        armControl.claw(gamepad2.dpad_up);
        armControl.control(gamepad2, true);
        armControl.throwPlane(gamepad2.dpad_down);
        armControl.enableAutoServo(gamepad2.a);
        armControl.moveLeft(gamepad2.left_bumper);
        armControl.moveRight(gamepad2.right_bumper);
    }
}
