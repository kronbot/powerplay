package org.firstinspires.ftc.teamcode.opmode.manual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Relative Manual Control", group = "Manual")
public class
RelativeManualControl extends OpMode {
    private final KronBot robot = new KronBot();
    private SampleMecanumDrive drive;

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double angle = robot.getCurentAngle();

        telemetry.addData("angle", angle);
        telemetry.update();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * Math.cos(Math.toRadians(angle)),
                        -gamepad1.left_stick_x * Math.sin(Math.toRadians(angle)),
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        if(gamepad1.a)
            robot.resetHeading();
    }
}