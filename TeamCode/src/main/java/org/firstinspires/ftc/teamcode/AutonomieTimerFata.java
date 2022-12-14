package org.firstinspires.ftc.teamcode;

import static java.lang.StrictMath.max;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomie Timer Fata")
public class AutonomieTimerFata extends LinearOpMode {
    KronBot Robot = new KronBot();
    ElapsedTime timer = new ElapsedTime();
    double acc = 0.55;
    double speed = 0.60;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        Robot.intakeServo.setPosition(0);
        Robot.armServo.setPosition(1);
        waitForStart();

        DriveAcc(1, 1, 1, 1, speed, 1.32, acc, 0.02);
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
