package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotControl;
import org.firstinspires.ftc.teamcode.utils.SlideControl;
import org.firstinspires.ftc.teamcode.utils.SlideLevelControl;
import org.firstinspires.ftc.teamcode.utils.Utils;

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
        slideLevelControl = new SlideLevelControl(robot);
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
            move |= robotControl.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            move |= robotControl.translate(gamepad1.left_trigger, gamepad1.right_trigger);
        }

        if (!move)
            robotControl.stop();


        slideControl.slide(gamepad1.left_bumper, gamepad1.right_bumper);
        slideControl.arm(gamepad1.y, gamepad1.a);
        slideControl.intake(gamepad1.x, gamepad1.b);


        telemetry.addData("Position", robot.getSlidePosition());
        slideLevelControl.updateState(gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left, gamepad1.dpad_down);
        // checking if the current state is finished
        if (!robot.isSlideMotorBusy() && slideLevelControl.getCurrentState() != Utils.State.REST)
            slideLevelControl.setCurrentState(Utils.State.REST);
    }
}
