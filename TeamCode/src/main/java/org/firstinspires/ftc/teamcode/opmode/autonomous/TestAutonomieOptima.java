package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "Left autonomous", group = "Autonomous")
public class TestAutonomieOptima extends LinearOpMode {
    private KronBot robot;
    private SlideControl slideControl;
 //   private SlideControlRunnable slideControlRunnable;

    private TagDetection tagDetection;
    private AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
//        slideControlRunnable = new SlideControlRunnable(slideControl);
//        Thread slideControlThread = new Thread(slideControlRunnable);

        tagDetection = new TagDetection(hardwareMap, telemetry);



        if (isStopRequested()) return;

//        slideControlThread.start();

        /* while (!isStarted() && !isStopRequested()) {
            tagOfInterest = tagDetection.detectTag();
            if (tagOfInterest != null) {
                telemetry.addData("tag", tagOfInterest.id);
                telemetry.update();
            }
        } */

        robot.controlIntake(0.0);

        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(90.00)), Math.toRadians(45.00))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(90)))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(90)))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(90)))
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .build();


        drive.followTrajectorySequence(test);
        //robot.controlIntake(1.0);

//        drive.followTrajectorySequence(goToCones2);
//        sleep(200);
//        robot.controlIntake(0.0);
//        sleep(500);
//        slideControl.setCoordinate(firstConeCoordinate + 750);
//        sleep(200);
//        drive.followTrajectorySequence(goToJunction);

        /*if(tagOfInterest != null) {
            if (tagOfInterest.id == 1)
                drive.followTrajectorySequence(parkingPos1);
            else if (tagOfInterest.id == 2)
                drive.followTrajectorySequence(parkingPos2);
            else
                drive.followTrajectorySequence(parkingPos3);
        } else
            drive.followTrajectorySequence(parkingPos2);*/


        while (!isStopRequested() && opModeIsActive());
//            drive.update();

//        slideControlRunnable.stop();
    }
}
