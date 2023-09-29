package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.openftc.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "ConePickup", group = "Autonomous")
public class ConePickup extends LinearOpMode {
    private KronBot robot;
    private SlideControl slideControl;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
        if (isStopRequested()) return;

        slideControl.setCoordinate(3800);
        sleep(200);
        robot.controlArm(0.0);
        sleep(200);
        robot.controlArm(1.0);
        sleep(200);
//        robot.controlIntake(1.0);
//        slideControl.setCoordinate(3500);
//        sleep(200);
//        robot.controlIntake(0.0);
//        robot.controlArm(0);
//        sleep(100);
//        robot.controlIntake(1);
//        sleep(100);
//        robot.controlArm(1);
        while (!isStopRequested() && opModeIsActive());
    }
}
