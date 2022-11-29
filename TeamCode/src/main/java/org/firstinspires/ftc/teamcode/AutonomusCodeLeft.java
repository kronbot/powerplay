package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveTest;
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
        SampleMecanumDriveTest drive = new SampleMecanumDriveTest(robot.hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d liftPose = new Pose2d(-30, 48, Math.toRadians(0));
        Pose2d placePose = new Pose2d(5, 55, Math.toRadians(45));
        double liftEndHeading = 180;
        double placeEndHeading = 45;

        waitForStart();

        if (isStopRequested()) return;
        TrajectorySequenceBuilder tsb = drive.trajectorySequenceBuilder(startPose)
                /* TODO: Add getting ready for placing cone */
                .splineToSplineHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(90)) /* middle position used for making the rotations without hitting a junction*/
                .splineToSplineHeading(placePose, Math.toRadians(placeEndHeading)); /* go to place position for the initial lifted cone*/
        /* TODO: Place the cone*/

        tsb = autonomusUtil.AutonomusCone(tsb, 4, liftPose, placePose, liftEndHeading, placeEndHeading);
        /* TODO: Add getting ready for lifting the cone */
        tsb.splineToSplineHeading(liftPose, Math.toRadians(180)) /* go to liftPose so we can have the last cone picked-up for manual control */
                .setReversed(false);

        //TODO: Add parking using autonomusUtil.AutonomusPark
        TrajectorySequence ts = tsb.build();

        drive.followTrajectorySequence(ts);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
