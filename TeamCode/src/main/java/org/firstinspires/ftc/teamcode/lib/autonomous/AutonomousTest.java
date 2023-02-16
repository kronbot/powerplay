package org.firstinspires.ftc.teamcode.lib.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(name = "Autonomous Test", group = "Autonomous")
public class AutonomousTest extends LinearOpMode {

    private KronBot robot;
    private SlideControl slideControl;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id <= 3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }
        }
        int TagID = 0;
        if (tagOfInterest != null) {
            TagID = tagOfInterest.id;
        }
        waitForStart();

        if (isStopRequested()) return;
        sleep(2000);
        robot.controlIntake(0.0);
        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(40,0))
                .addTemporalMarker(0.3, () -> {
                    slideControl.setState(SlideControl.State.THIRD);
                })
                .lineToSplineHeading(new Pose2d(62,-5,Math.toRadians(-30)))
                .build();

        TrajectorySequence GetCone = drive.trajectorySequenceBuilder(new Pose2d(62, -5, Math.toRadians(-30)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(58,16,Math.toRadians(95)))
                .addTemporalMarker(0.5, () -> {
                    slideControl.setState(SlideControl.State.GROUND);
                })
           .build();

        TrajectorySequence ToJunk = drive.trajectorySequenceBuilder(new Pose2d(58, 16, Math.toRadians(95)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(61,-6,Math.toRadians(-30)))
                .addTemporalMarker(0.1,() -> {
                slideControl.setState(SlideControl.State.THIRD);
                })
                .build();

        TrajectorySequence GetCone1 = drive.trajectorySequenceBuilder(new Pose2d(61, -6, Math.toRadians(-30)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(58,16,Math.toRadians(95)))
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
      drive.followTrajectorySequence(FirstCone);
      sleep(500);
      robot.controlIntake(1.0);
      drive.followTrajectorySequence(GetCone);
      sleep(200);
      robot.controlIntake(0.0);
        slideControl.setState(SlideControl.State.GROUND);
        robot.controlIntake(1.0);
        drive.followTrajectorySequence(ToJunk);
        sleep(500);
        robot.controlIntake(1.0);
        drive.followTrajectorySequence(GetCone1);
        sleep(200);
        robot.controlIntake(1.0);
        if(TagID==2)
        {
            drive.followTrajectorySequence(TagID2);
        }
        else
        {
            drive.followTrajectorySequence(TagID2);
            if(TagID==1)
                drive.followTrajectorySequence(TagID1);
            else
                drive.followTrajectorySequence(TagID3);
        }
        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            telemetry.addData("AprilTagID", TagID);
        }
    }
}
