package org.firstinspires.ftc.teamcode;

import static java.lang.StrictMath.max;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Autonomie camera")
public class AutonomieCamera extends LinearOpMode
{
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

    KronBot Robot = new KronBot();
    ElapsedTime timer = new ElapsedTime();
    double acc = 0.55;
    double speed = 0.60;

    @Override
    public void runOpMode()
    {
        initHardwareMap();
        Robot.intakeServo.setPosition(0);
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
                    if(1<=tag.id && tag.id <=3)
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(tagOfInterest.id == 1){
            DriveAcc(-1, 1,1,-1,speed,1.95,acc,0.02);

            while (timer.seconds() < 3 && opModeIsActive()) ;
            if (!opModeIsActive())
                return;

            DriveAcc(1, 1, 1, 1, speed, 1.4, acc, 0.02);
        }
        else if(tagOfInterest.id == 2){
            DriveAcc(1, 1, 1, 1, speed, 1.4, acc, 0.02);
        }else{
            DriveAcc(1, -1,-1,1,speed,1.8,acc,0.02);

            timer.reset();
            while (timer.seconds() < 3 && opModeIsActive()) ;
            if (!opModeIsActive())
                return;

            DriveAcc(1, 1, 1, 1, speed, 1.4, acc, 0.02);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void DriveAcc(double bl, double br, double fl, double fb, double power, double runTime, double acc, double add) {
        timer.reset();

        while (timer.seconds() < runTime - runTime / 4) {
            acc = max(acc - add, 0);
            Robot.drive(bl, br, fl, fb, power - acc);
            if (!opModeIsActive())
                return;
        }
        while (timer.seconds() < runTime) {
            acc = Math.min(power - 0.2, acc + add);
            Robot.drive(bl, br, fl, fb, power - acc);
            if (!opModeIsActive())
                return;
        }

        Robot.drive(bl, br, fl, fb, 0);
    }

    void initHardwareMap() {
        Robot.initHardwareMap(hardwareMap);
        Robot.resetSlideEncoder();

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}