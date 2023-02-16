package org.firstinspires.ftc.teamcode.lib.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(name = "Autonomous Test", group = "Autonomous")
public class AutonomousTest extends LinearOpMode {

    private KronBot robot;
    private SlideControl slideControl;

    private TagDetection tagDetection;
    private AprilTagDetection tagOfInterest = null;

    public static Integer FIRST_CONE_COORDINATE = 700;

    public static double firstConeX = 55;
    public static double firstConeY = -5;
    public static double firstConeAngle = -35;

    public static double backwardCoordinateX = 53;
    public static double backwardCoordinateY = 0;

    public static double ConeX = 55.3;
    public static double ConeY = 17;
    public static double ConeAngle = 95;


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

        TrajectorySequence GetCone = drive.trajectorySequenceBuilder(new Pose2d(firstConeX, firstConeY, Math.toRadians(firstConeAngle)))
                .setReversed(true)
                .lineTo(new Vector2d(backwardCoordinateX,backwardCoordinateY))
                .addTemporalMarker(0.5, () -> {
                    slideControl.setCoordinate(FIRST_CONE_COORDINATE);
                })
                .lineToSplineHeading(new Pose2d(ConeX,ConeY,Math.toRadians(ConeAngle)))
                .build();

        TrajectorySequence ToJunk = drive.trajectorySequenceBuilder(new Pose2d(56, 18, Math.toRadians(95)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(57,-5,Math.toRadians(-30)))
                .addTemporalMarker(0.1,() -> {
                slideControl.setState(SlideControl.State.THIRD);
                })
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

        robot.controlIntake(0.0);
        drive.followTrajectorySequence(InitialCone);
        sleep(500);
        robot.controlIntake(1.0);
        drive.followTrajectorySequence(GetCone);
        sleep(200);
        robot.controlIntake(1.0);
//        drive.followTrajectorySequence(ToJunk);
//        sleep(500);
//        robot.controlIntake(1.0);
//        drive.followTrajectorySequence(GetCone1);
//        sleep(200);
        robot.controlIntake(1.0);

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
