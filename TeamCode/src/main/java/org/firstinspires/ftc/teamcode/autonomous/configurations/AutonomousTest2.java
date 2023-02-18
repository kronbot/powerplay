package org.firstinspires.ftc.teamcode.autonomous.configurations;

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
@Autonomous(name = "Parking", group = "Autonomous")
public class AutonomousTest2 extends LinearOpMode {

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

        TrajectorySequence TagID2 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(51,0))
                .build();
        TrajectorySequence TagID1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d( 51, 0))
                .lineTo(new Vector2d(0,15))
                .build();
        TrajectorySequence TagID3 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(50,0))
                .lineTo(new Vector2d(50,-20))
                .build();

        while (!isStarted() && !isStopRequested()) {
            tagOfInterest = tagDetection.detectTag();
        }
        while (!isStopRequested() && opModeIsActive()) {
        }
        if(tagOfInterest!=null)
        {
            if (tagOfInterest.id == 1) {
                drive.followTrajectorySequence(TagID1);
            } else if (tagOfInterest.id == 2) {
                drive.followTrajectorySequence(TagID2);
            } else {
                drive.followTrajectorySequence(TagID3);
            }
        } else {
            drive.followTrajectorySequence(TagID2);
        }

    }
}
