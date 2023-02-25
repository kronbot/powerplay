package org.firstinspires.ftc.teamcode.opmode.manual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
@TeleOp(name="Relative Manual Control")
public class
RelativeManualControl extends OpMode {
    private final KronBot robot = new KronBot();
    private SampleMecanumDrive drive;
    private double angle = 0.0;

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        angle = robot.GetCurentAngle();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * Math.cos((angle * Math.PI) / 180),
                        -gamepad1.left_stick_x * Math.sin((angle * Math.PI) / 180),
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        if(gamepad1.a)
            robot.resetHeading();
    }
}