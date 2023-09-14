package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.CameraControl;

@Config
@Autonomous(name = "Camera", group = "Autonomous")
public class Camera extends LinearOpMode {
    private KronBot robot;
    private CameraControl cameraControl;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        cameraControl = new CameraControl(robot, telemetry);

        while (!opModeIsActive())
            telemetry.addData("Position", cameraControl.detect());

        waitForStart();
    }
}
