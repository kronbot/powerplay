package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.AutonomusUtil;

@Autonomous(name = "Autonomus Code for Left")
public class AutonomusCodeLeft extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private final AutonomusUtil autonomusUtil = new AutonomusUtil();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap();
        SampleMecanumDrive drive = new SampleMecanumDrive(robot.hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d liftPose = new Pose2d(-30, 48, Math.toRadians(0));
        Pose2d placePose = new Pose2d(5, 55, Math.toRadians(45));
        double liftEndHeading = 180;
        double placeEndHeading = 45;
        waitForStart();

        if (isStopRequested()) return;
        //TODO: Set start pose and add code for doing it for all start positions
        TrajectorySequenceBuilder tsb = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(placePose, Math.toRadians(placeEndHeading));

        tsb = autonomusUtil.AutonomusCone(tsb, 4, liftPose, placePose, liftEndHeading, placeEndHeading);

        tsb.splineToSplineHeading(liftPose, Math.toRadians(180));
        //TODO: Add parkking using autonomousUtil.AutonomusPark
        TrajectorySequence ts = tsb.build();

        drive.followTrajectorySequence(ts);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
