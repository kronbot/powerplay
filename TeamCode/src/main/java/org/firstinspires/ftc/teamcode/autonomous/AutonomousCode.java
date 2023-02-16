package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.autonomous.configurations.TestConfiguration;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.firstinspires.ftc.teamcode.lib.Utils;
import org.firstinspires.ftc.teamcode.lib.autonomous.GlobalCoordinatePosition;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous
public class AutonomousCode extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private final TestConfiguration configuration = new TestConfiguration();
    private SlideControl slideControl;

    private TagDetection tagDetection;
    private AprilTagDetection tagOfInterest = null;

    private GlobalCoordinatePosition position;

    private double error;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);

        slideControl = new SlideControl(robot, telemetry);

        tagDetection = new TagDetection(hardwareMap, telemetry);

        position = new GlobalCoordinatePosition(
            robot.leftEncoder,
            robot.rightEncoder,
            robot.frontEncoder,
            configuration,
            telemetry
        );

        while (!isStarted() && !isStopRequested()) {
            tagOfInterest = tagDetection.detectTag();
//            if (tagOfInterest != null)
//                tagDetection.tagToTelemetry(tagOfInterest);
        }

        Thread positionThread = new Thread(position);
        positionThread.start();

//        Thread slideThread = new Thread((RunslideControl);

//        slideControl.setCoordinate(1000);

        linear(180);


        Thread.sleep(500);
        rotate(Math.PI / 4);
        Thread.sleep(500);

        //ridica glisiera

        linear(10);
        Thread.sleep(500);

        // pune con

        linear(-10);
        Thread.sleep(500);

        // coboara glisiera

        rotate(Math.PI / 4 * 3);
        Thread.sleep(500);
        linear(150);
        Thread.sleep(500);

        // ia con



        while (opModeIsActive()) {
//            rotate(Math.PI / 2);
//            Thread.sleep(500);
        }

        position.stop();
    }

    public void linear(double distance) {
        final double targetX = position.getX() + Math.sin(position.getAngle()) * distance;
        final double targetY = position.getY() + Math.cos(position.getAngle()) * distance;
        final double targetAngle = position.getAngle();

        double lastAngleError = 0, lastError = 0;
        double angleIntegralSum = 0, integralSum = 0;
        double angleError = 0;
        error = Utils.distance(position.getX(), position.getY(), targetX, targetY);

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

            double anglePower = angleError * (backwards ?
                    configuration.getBackwardAngleOffset() :
                    configuration.getForwardAngleOffset()) *
                    (distance > 0 ? 1 : -1);

            double leftPower = distancePower + anglePower;
            double rightPower = distancePower - anglePower;
            if (leftPower > 1.0)
                leftPower = 1.0;
            if (rightPower > 1.0)
                rightPower = 1.0;

            leftPower = Utils.map(leftPower, 0, 1, 0, configuration.getMaxSpeed());
            rightPower = Utils.map(rightPower, 0, 1, 0, configuration.getMaxSpeed());

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
            if (power > 1.0)
                power = 1.0;
            power = Utils.map(power, 0, 1, 0, configuration.getMaxSpeed());

            robot.drive(power, -power, power, -power, 1);

            lastError = error;
            timer.reset();
        }

        robot.stopMotors();
    }
}
