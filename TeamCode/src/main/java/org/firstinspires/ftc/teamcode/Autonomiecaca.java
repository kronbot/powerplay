package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(name = "random")
public class Autonomiecaca extends LinearOpMode{
    @Override
    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);
        Trajectory FirstCone = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(40,5))
                .build();
        Trajectory FirstCone2 = drive.trajectoryBuilder(new Pose2d(40,5,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(60, 10, Math.toRadians(50)))
                .build();
        waitForStart();
        drive.followTrajectory(FirstCone);
        drive.followTrajectory(FirstCone2);

        Pose2d poseEstimate = drive.getPoseEstimate();

        while(opModeIsActive())
        {
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }


        while (!isStopRequested() && opModeIsActive()) {}
    }
}