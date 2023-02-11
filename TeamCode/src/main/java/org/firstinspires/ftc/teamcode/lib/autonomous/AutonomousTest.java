package org.firstinspires.ftc.teamcode.lib.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        robot.initHardwareMap(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

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
        int TagID=0;
        if(tagOfInterest!=null)
        {
            TagID=tagOfInterest.id;
        }
        if (isStopRequested()) return;
        robot.controlIntake(1.0);
        TrajectorySequence FirstCone= drive.trajectorySequenceBuilder(new Pose2d(-35,-70,Math.toRadians(90)))
                .lineTo(new Vector2d(-35,-20))
                .addDisplacementMarker(() -> {
                  slideControl.setState(SlideControl.State.THIRD);
                })
                .splineToSplineHeading(new Pose2d(-30, -5, Math.toRadians(45)),Math.toRadians(45))
                .build();
        drive.followTrajectorySequence(FirstCone);
        slideControl.setState(SlideControl.State.GROUND);
             robot.controlIntake(0.0);
        sleep(2000);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        TrajectorySequence GetCone=drive.trajectorySequenceBuilder(new Pose2d(-30 -5, Math.toRadians(50)))
                .setReversed(true)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-60,-12,Math.toRadians(180)),Math.toRadians(180))
                .addTemporalMarker(1,() -> {

                    slideControl.setState(SlideControl.State.FIVE);
                })
                .addDisplacementMarker(()-> {
                    robot.controlIntake(1.0);
                })
                .build();

        sleep(2000);
        drive.followTrajectorySequence(GetCone);
        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        TrajectorySequence ToJunk=drive.trajectorySequenceBuilder(new Pose2d(-60 -12,Math.toRadians(180)))
                .setReversed(true)
                .addTemporalMarker(1,() -> {
                    slideControl.setState(SlideControl.State.THIRD);
                })
                .splineToSplineHeading(new Pose2d(-30,-5,Math.toRadians(45)),Math.toRadians(45))
                .build();
        sleep(2000);
        drive.followTrajectorySequence(ToJunk);
        slideControl.setState(SlideControl.State.GROUND);
        robot.controlIntake(0.0);
        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        telemetry.addData("AprilTagID",TagID);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
