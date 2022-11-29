package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class SlideEncoderTestAutonomous extends LinearOpMode {
    KronBot robot = new KronBot();

    @Override
    public void runOpMode() {
        robot.initHardwareMap(hardwareMap);

        robot.slideDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            robot.slideDc.setTargetPosition(5000);
            robot.slideDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slideDc.setPower(0.1);

            while (opModeIsActive() && robot.slideDc.isBusy()) {
                telemetry.addData("Current position", robot.slideDc.getCurrentPosition());
                telemetry.update();
            }
        }

        robot.slideDc.setPower(0);
        robot.slideDc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}