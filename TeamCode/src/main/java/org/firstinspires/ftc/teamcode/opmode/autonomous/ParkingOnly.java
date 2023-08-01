package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "Parking Only", group = "Autonomous")
public class ParkingOnly extends LinearOpMode {
    private KronBot robot;
    private SlideControl slideControl;

    private double acc = 0.55;
    private double speed = 0.60;
    ElapsedTime timer = new ElapsedTime();
    private TagDetection tagDetection;
    private AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
        tagDetection = new TagDetection(hardwareMap, telemetry);
        if (isStopRequested()) return;
        while (!isStarted() && !isStopRequested()) {
            tagOfInterest = tagDetection.detectTag();
            if (tagOfInterest != null) {
                telemetry.addData("tag", tagOfInterest.id);
                telemetry.update();
            }
        }
        sleep(200);
        robot.controlIntake(0.0);
        sleep(200);
        slideControl.setCoordinate(150);
        sleep(200);
        if (tagOfInterest != null)
        {
            if (tagOfInterest.id == 1)
            {
                DriveAcc(1, 1, 1, 1, speed, 1.4, acc, 0.02);
                while (timer.seconds() < 3 && opModeIsActive()) ;
                if (!opModeIsActive())
                    return;
                DriveAcc(-1, 1, 1, -1, speed, 1.5, acc, 0.02);
            }
            else if (tagOfInterest.id == 2)
            {
                DriveAcc(1, 1, 1, 1, speed, 1.4, acc, 0.02);
            }
            else {
                DriveAcc(1, 1,1,1,speed,1.4,acc,0.02);

                timer.reset();
                while (timer.seconds() < 3 && opModeIsActive()) ;
                if (!opModeIsActive())
                    return;
                DriveAcc(1, -1, -1, 1, speed, 1.5, acc, 0.02);
            }

        }
        }
        void DriveAcc(double fl, double fr, double bl, double br, double power, double runTime, double acc, double add) {
            timer.reset();

            while (timer.seconds() < runTime - runTime / 4) {
                acc = max(acc - add, 0);
                robot.drive(fl, fr, bl, br, power - acc);
                if (!opModeIsActive())
                    return;
            }
            while (timer.seconds() < runTime) {
                acc = Math.min(power - 0.2, acc + add);
                robot.drive(fl, fr, bl, br, power - acc);
                if (!opModeIsActive())
                    return;
            }
            robot.drive(fl, fr, bl, br, 0);
        }
    }