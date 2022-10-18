package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Autonomus Code")
public class AutonomusCode extends LinearOpMode {
    Kronbot Robot = new Kronbot();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.initHardwareMap();
        SampleMecanumDrive drive = new SampleMecanumDrive(Robot.hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
            drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-35, -25, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -5, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(1)
                .build()
        );

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
