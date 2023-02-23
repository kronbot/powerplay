package org.firstinspires.ftc.teamcode.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
@TeleOp(name="relativeManualControl")
public class
RelativeManualControl extends OpMode {
    private final KronBot robot=new KronBot();
    private final RobotControl robotControl = new RobotControl(robot,telemetry);
    private final SlideControl slideControl = new SlideControl(robot,telemetry);
    private double Angle=0.0;
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.resetSlideEncoder();
        robot.controlIntake(1);
    }

    @Override
    public void loop() {
        Angle=robot.GetCurentAngle();
        boolean move = robotControl.rotate(gamepad1.right_stick_x/1.25);
        if (!move)
            move = robotControl.drive(gamepad1.left_stick_x*Math.sin((Angle*Math.PI)/180), -gamepad1.left_stick_y*Math.cos((Angle*Math.PI)/180));
        if (!move)
            robotControl.stop();
//        robotControl.debug();
        if(gamepad1.a)
            robot.resetHeading();
        slideControl.intake(gamepad2.dpad_up);
        slideControl.control(gamepad2, false);
    }
}
