package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(name = "Left side autonomous", group = "Autonomous")
public class LeftSideAutonomous extends LinearOpMode {

    private KronBot robot;
    private SlideControl slideControl;

    private TagDetection tagDetection;
    private AprilTagDetection tagOfInterest = null;

    public static Integer FIRST_CONE_COORDINATE = 600;

    public static double firstConeX = 56.5;
    public static double firstConeY = -2.5;
    public static double firstConeAngle = -33;

    public static double backwardCoordinateX = 45;
    public static double backwardCoordinateY = 0;

    public static double coneX = 57.5;
    public static double coneY = 20.5;
    public static double coneAngle = 95;

    public static double junctionX = 45;
    public static double junctionY = -18;
    public static double junctionAngle = -15;

    public static double junctionForwardX = 52.8;


    @Override
    public void runOpMode() {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
        tagDetection = new TagDetection(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (isStopRequested()) return;

        TrajectorySequence InitialCone = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(40,0))
                .addTemporalMarker(0.3, () -> {
                    slideControl.setState(SlideControl.State.THIRD);
                })
                .lineToSplineHeading(new Pose2d(firstConeX,firstConeY,Math.toRadians(firstConeAngle)))
                .build();

        TrajectorySequence GetNewCone = drive.trajectorySequenceBuilder(new Pose2d(firstConeX, firstConeY, Math.toRadians(firstConeAngle)))
                .setReversed(true)
                .lineTo(new Vector2d(backwardCoordinateX,backwardCoordinateY))
                .addTemporalMarker(0.5, () -> {
                    slideControl.setCoordinate(FIRST_CONE_COORDINATE);
                })
                .lineToSplineHeading(new Pose2d(coneX, coneY, Math.toRadians(coneAngle)))
                .build();

        TrajectorySequence MoveToJunction = drive.trajectorySequenceBuilder(new Pose2d(coneX, coneY, Math.toRadians(coneAngle)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(junctionX, junctionY, Math.toRadians(junctionAngle)))
                .addTemporalMarker(0.3, () -> {
                    slideControl.setState(SlideControl.State.THIRD);
                })
                .lineTo(new Vector2d(junctionForwardX, junctionY))
                .build();

        TrajectorySequence GetCone1 = drive.trajectorySequenceBuilder(new Pose2d(56.5, -5, Math.toRadians(-30)))
                .setReversed(true)
                .lineTo(new Vector2d(53,0))
                .lineToSplineHeading(new Pose2d(55.3,17,Math.toRadians(95)))
                .addTemporalMarker(0.5, () -> {
                    slideControl.setState(SlideControl.State.GROUND);
                })
                .build();
        TrajectorySequence TagID2 = drive.trajectorySequenceBuilder(new Pose2d(61, -6, Math.toRadians(-30)))
                .lineTo(new Vector2d(0,56))
                .build();
        TrajectorySequence TagID1 = drive.trajectorySequenceBuilder(new Pose2d(61, -6, Math.toRadians(-30)))
                .lineTo(new Vector2d(-15,56))
                .build();
        TrajectorySequence TagID3 = drive.trajectorySequenceBuilder(new Pose2d(61, -6, Math.toRadians(-30)))
                .lineTo(new Vector2d(15,56))
                .build();

        while (!isStarted() && !isStopRequested()) {
            tagOfInterest = tagDetection.detectTag();
//            if (tagOfInterest != null)
//                tagDetection.tagToTelemetry(tagOfInterest);
        }

        robot.controlIntake(0);
        drive.followTrajectorySequence(InitialCone);
        sleep(200);
        robot.controlIntake(1);
        drive.followTrajectorySequence(GetNewCone);
        robot.controlIntake(0);
        sleep(500);
        slideControl.setCoordinate(FIRST_CONE_COORDINATE + 750);
        sleep(500);
        drive.followTrajectorySequence(MoveToJunction);
        robot.controlIntake(1);
//        sleep(500);
//        robot.controlIntake(1.0);
//        drive.followTrajectorySequence(GetCone1);
//        sleep(200);
//        robot.controlIntake(1.0);

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
//            telemetry.addData("AprilTagID", tagOfInterest.id);
        }

//        if(TagID==2)
//        {
//            drive.followTrajectorySequence(TagID2);
//        }
//        else
//        {
//            drive.followTrajectorySequence(TagID2);
//            if(TagID==1)
//                drive.followTrajectorySequence(TagID1);
//            else
//                drive.followTrajectorySequence(TagID3);
//        }
    }
}
