package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Util;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Objects;

//@TeleOp(name = "ManualControlHasisi")
public class ManualControlVechi extends OpMode {
    KronBot robot = new KronBot();
    double EPS = 0.1;
    double ACC = 0.05;
    double currentPowerRunning;
    double currentPowerRotation;
    boolean intakeActivated = false;
    boolean intakeButtonPressed = false;

    boolean ruletaActivated = false;
    boolean ruletaButtonPressed = false;
    boolean ruletaActivatedBACK = false;
    boolean ruletaButtonPressedBACK = false;

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);

        robot.frontLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void driveWheels() {
        if (gamepad1.left_trigger > EPS || gamepad1.right_trigger > EPS || Math.abs(gamepad1.right_stick_x) > EPS) {
            currentPowerRunning = 0;

            double power;

            if (gamepad1.left_trigger > EPS || gamepad1.right_stick_x < -EPS)
                power = Math.max(gamepad1.left_trigger, -gamepad1.right_stick_x);
            else
                power = Math.max(gamepad1.right_trigger, gamepad1.right_stick_x);

            if (currentPowerRotation == 0)
                currentPowerRotation = 0.1;
            else
                currentPowerRotation = Math.min(currentPowerRotation + ACC, power);

            if (gamepad1.left_trigger > EPS || gamepad1.right_stick_x < -EPS)
                robot.drive(-1, 1, -1, 1, currentPowerRotation);
            else
                robot.drive(1, -1, 1, -1, currentPowerRotation);
        } else {
            currentPowerRotation = 0;

            double xvalue = gamepad1.left_stick_x, yvalue = -gamepad1.left_stick_y;
            double angle = Math.toDegrees(Math.atan2(yvalue, xvalue));
            double power = Math.sqrt(xvalue * xvalue + yvalue * yvalue);

            if (power < EPS)
                currentPowerRunning = 0;
            else if (currentPowerRunning < power)
                currentPowerRunning = Math.min(currentPowerRunning + ACC, power);
            else
                currentPowerRunning = power;

            if (EPS < yvalue) {
                if (angle < 90) {
                    double Dial1WheelsPower = Utils.map(angle, 0, 90, -1, 1);
                    if (Dial1WheelsPower < EPS)
                        Dial1WheelsPower = 0;
                    robot.drive(1, Dial1WheelsPower, Dial1WheelsPower, 1, currentPowerRunning);
                } else {
                    double Dial2WheelsPower = Utils.map(angle, 90, 180, 1, -1);
                    if (Dial2WheelsPower < EPS)
                        Dial2WheelsPower = 0;
                    robot.drive(Dial2WheelsPower, 1, 1, Dial2WheelsPower, currentPowerRunning);
                }
            } else if (yvalue < -EPS) {
                if (angle < -90) {
                    double Dial3WheelsPower = Utils.map(angle, -180, -90, 1, -1);
                    if (Dial3WheelsPower > -EPS)
                        Dial3WheelsPower = 0;
                    robot.drive(-1, Dial3WheelsPower, Dial3WheelsPower, -1, currentPowerRunning);
                } else {
                    double Dial4WheelsPower = Utils.map(angle, -90, 0, -1, 1);
                    if (Dial4WheelsPower > -EPS)
                        Dial4WheelsPower = 0;
                    robot.drive(Dial4WheelsPower, -1, -1, Dial4WheelsPower, currentPowerRunning);
                }
            } else {
                if (-EPS < xvalue && xvalue < EPS)
                    robot.stopMotors();
                else if (xvalue > EPS)
                    robot.drive(1, -1, -1, 1, currentPowerRunning);
                else
                    robot.drive(-1, 1, 1, -1, currentPowerRunning);
            }
        }
    }

    @Override
    public void loop() {
        driveWheels();
    }
}