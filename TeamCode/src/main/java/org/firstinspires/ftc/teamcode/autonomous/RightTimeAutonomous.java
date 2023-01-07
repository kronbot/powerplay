package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.lib.AutonomousBuilder;
import org.firstinspires.ftc.teamcode.lib.PowerplayTimeAutonomyConfiguration;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TimeAutonomyConfiguration;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RightTimeAutonomous extends LinearOpMode {
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;

    // UNITS ARE METERS
    private final double tagsize = 0.166;

    private final int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    private AprilTagDetection tagOfInterest = null;

    private final KronBot robot = new KronBot();
    private final TimeAutonomyConfiguration configuration = new PowerplayTimeAutonomyConfiguration();
    private final AutonomousBuilder autonomousBuilder = new AutonomousBuilder(this, configuration, robot);

    @Override
    public void runOpMode() throws InterruptedException {

        autonomousBuilder.initialize(hardwareMap);
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
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id <3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

            }

            telemetry.update();
            sleep(20);
        }

        autonomousBuilder.toggleIntake();
        autonomousBuilder.setSlideState(SlideControl.State.THIRD);
        autonomousBuilder.forward(0.63);
        autonomousBuilder.delay(1.0);
        autonomousBuilder.translateLeft(1.6);
        autonomousBuilder.delayUntilSlideIsResting();
        autonomousBuilder.toggleIntake();
        autonomousBuilder.rotateCounterClockwise(0.05);

        if (tagOfInterest != null) {
            if (tagOfInterest.id == 1)
                autonomousBuilder.translateRight(0.5);
            else if (tagOfInterest.id == 2)
                autonomousBuilder.translateRight(1.4);
            else
                autonomousBuilder.translateRight(2.5);
        } else
            autonomousBuilder.translateRight(2.5);

        autonomousBuilder.uninitialize();
    }

    private void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
