package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.autonomous.configurations.TestConfiguration;
import org.firstinspires.ftc.teamcode.lib.autonomous.GlobalCoordinatePosition;

@Autonomous
public class BuciRenewed extends LinearOpMode {
    private KronBot robot;
    private GlobalCoordinatePosition position;

    @Override
    public void runOpMode() {
        robot.initHardwareMap(hardwareMap);
        position = new GlobalCoordinatePosition(
                robot.leftEncoder,
                robot.rightEncoder,
                robot.frontEncoder,
                new TestConfiguration()
        );


        waitForStart();

        Thread yeees = new Thread(position);
        yeees.start();

        while (opModeIsActive()) {
            telemetry.addData("x", position.getX());
            telemetry.addData("y", position.getY());
            telemetry.addData("angle", position.getAngle());
            telemetry.update();
        }
    }
}
