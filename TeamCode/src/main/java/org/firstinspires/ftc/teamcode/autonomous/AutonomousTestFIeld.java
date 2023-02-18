package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class AutonomousTestFIeld extends LinearOpMode {
    private final KronBot robot = new KronBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);

        waitForStart();

        robot.drive(1, 1, 1, 1, 1);
        sleep(500);
        robot.stopMotors();

        while (opModeIsActive()) {};
    }
}
