package org.firstinspires.ftc.teamcode.opmode.manual;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.Utils;

@Config
@TeleOp(name = "New Dual Manual Control", group = "Manual")
public class RoadrunnerManualControlTranslation extends OpMode {
    private final KronBot robot;
    private final SlideControl slideControl;

    private SampleMecanumDrive drive;

    public static double minMidPower = 0.25;
    public static double maxMidPower = 0.65;
    public static double maxPower = 0.75;
    public static double deadZoneMultiplier = 0.25;

    public static double translationPower;

    public RoadrunnerManualControlTranslation() {
        robot = new KronBot();
        slideControl = new SlideControl(robot, telemetry);
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.resetSlideEncoder();
        robot.controlIntake(1);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger != 0.0)
            translationPower = gamepad1.left_trigger;
        else if (gamepad1.right_trigger != 0.0)
            translationPower = -gamepad1.right_trigger;
        else
            translationPower = 0.0;

        drive.setWeightedDrivePower(
                new Pose2d(
                        motorPower(-gamepad1.left_stick_y),
                        motorPower(translationPower),
//                        motorPower(-gamepad1.left_stick_x),
                        motorPower(-gamepad1.right_stick_x)
                )
        );

        drive.update();

//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//        telemetry.update();

        slideControl.intake(gamepad2.dpad_up);
        slideControl.control(gamepad2, false);
    }

    private double motorPower(double power) {
        if (-Utils.EPS * deadZoneMultiplier < power && power < Utils.EPS * deadZoneMultiplier)
            return 0;
        else if (power < -0.9)
            return -maxPower;
        else if (power > 0.9)
            return maxPower;

        if (power < 0)
            return Utils.map(power, -1, 0, -maxMidPower, -minMidPower);
        return Utils.map(power, 0, 1, minMidPower, maxMidPower);
    }
}