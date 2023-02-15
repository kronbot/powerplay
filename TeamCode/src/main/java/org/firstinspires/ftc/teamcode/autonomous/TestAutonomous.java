package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.autonomous.configurations.TestConfiguration;
import org.firstinspires.ftc.teamcode.lib.Utils;
import org.firstinspires.ftc.teamcode.lib.autonomous.GlobalCoordinatePosition;

@Config
@Autonomous
public class TestAutonomous extends LinearOpMode {
    public static double DISTANCE = 100;
    public static double FORWARD_ANGLE_OFFSET = 0.025;
    public static double BACKWARDS_ANGLE_OFFSET = 0.01;
    private final KronBot robot = new KronBot();
    private final TestConfiguration configuration = new TestConfiguration();
    private GlobalCoordinatePosition position;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);
        position = new GlobalCoordinatePosition(
            robot.leftEncoder,
            robot.rightEncoder,
            robot.frontEncoder,
            configuration,
            telemetry
        );

        waitForStart();
        Thread positionThread = new Thread(position);
        positionThread.start();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                rotate(Math.PI);
                Thread.sleep(500);
            } else if (gamepad1.a) {
                rotate(Math.PI / 2);
                Thread.sleep(500);
            } else if (gamepad1.b) {
                Thread.sleep(1000);
                linear(DISTANCE);
                Thread.sleep(1000);
                rotate(Math.PI);
                Thread.sleep(1000);
                linear(DISTANCE);
                Thread.sleep(1000);
                rotate(Math.PI);
            } else if (gamepad1.y) {
                linear(DISTANCE);
                Thread.sleep(500);
            }
        }

        while (opModeIsActive()) {}
        position.stop();
    }

    public void linear(double distance) {
        final double targetX = position.getX() + Math.sin(position.getAngle()) * distance;
        final double targetY = position.getY() + Math.cos(position.getAngle()) * distance;
        final double targetAngle = position.getAngle();

        double lastAngleError = 0, lastError = 0;
        double angleError = 0, error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
        double angleIntegralSum = 0, integralSum = 0;

        double startX = position.getX();
        double startY = position.getY();

        boolean backwards = distance < 0;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && Math.abs(error) > configuration.getAcceptableError() &&
                Utils.distance(startX, startY, targetX, targetY) + configuration.getAcceptableError() >
                        Utils.distance(startX, startY, position.getX(), position.getY())) {
            integralSum += error;
            angleIntegralSum += angleError;
            double distanceDerivative = (error - lastError) / timer.seconds();
            double angleDerivative = (angleError - lastAngleError) / timer.seconds();

            double distancePower = (
                    (configuration.getKp() * error) +
                            (configuration.getKi() * integralSum) +
                            (configuration.getKd() * distanceDerivative)
            );

            double anglePower = angleError * (backwards ? BACKWARDS_ANGLE_OFFSET : FORWARD_ANGLE_OFFSET) * (distance > 0 ? 1 : -1);

            double leftPower = Utils.map(distancePower + anglePower, 0, 1, 0, configuration.getMaxSpeed());
            double rightPower = Utils.map(distancePower - anglePower, 0, 1, 0, configuration.getMaxSpeed());

            telemetry.addData("backwards", backwards);
            telemetry.addData("error", error);
            telemetry.addData("distance", Math.hypot(targetX - position.getX(), targetY - position.getY()));
            telemetry.addData("target x", targetX);
            telemetry.addData("target y", targetY);
            telemetry.addData("x", position.getX());
            telemetry.addData("y", position.getY());
            telemetry.addData("angle", position.getAngle());
            telemetry.addData("angle error", angleError);
            telemetry.addData("integral sum", integralSum);
            telemetry.addData("angle integral sum", angleIntegralSum);
            telemetry.addData("distance derivative", distanceDerivative);
            telemetry.addData("angle derivative", angleDerivative);
            telemetry.addData("distance power", distancePower);
            telemetry.addData("angle power", anglePower);
            telemetry.addData("left", leftPower);
            telemetry.addData("right", rightPower);
            telemetry.addData("power", configuration.getMaxSpeed());

            telemetry.update();

            robot.drive(leftPower, rightPower, leftPower, rightPower, backwards ? -1 : 1);

            lastAngleError = angleError;
            lastError = error;
            error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
            angleError = targetAngle - Math.atan2(position.getY(), position.getX());
            timer.reset();
        }

        robot.stopMotors();
    }

    public void rotate(double radians) {
        double target = position.getAngle() + radians;
        double lastError = 0, error = target - position.getAngle();
        double integralSum = 0;
        ElapsedTime timer = new ElapsedTime();
        boolean negative = radians < 0;
        while (
                Math.abs(error) > configuration.getAcceptableError() &&
                (negative && position.getAngle() > target ||
                position.getAngle() < target)
        ) {
            error = target - position.getAngle();

            integralSum += error;
            double derivative = (error - lastError) / timer.seconds();
            double power = (
                    (configuration.getAngleKp() * error) +
                            (configuration.getAngleKi() * integralSum) +
                            (configuration.getAngleKd() * derivative)
            );

            power = Utils.map(power, 0, 1, 0, configuration.getMaxSpeed());

            robot.drive(power, -power, power, -power, 1);

            lastError = error;
            timer.reset();
        }

        robot.stopMotors();
    }
}
