package org.firstinspires.ftc.teamcode.lib.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Utils;
import org.firstinspires.ftc.teamcode.util.Encoder;

// calculates the current position using encoders
public class GlobalCoordinatePosition implements Runnable {
    private final long UPDATE_TIME = 500; // in miliseconds
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;
    private final AutonomousConfiguration configuration;
    private final Telemetry telemetry;

    private double x = 0, y = 0, angle = 0;
    private double prevLeftPosition = 0, prevRightPosition = 0, prevFrontPosition = 0;
    private boolean isRunning = true;

    public GlobalCoordinatePosition(
            Encoder leftEncoder,
            Encoder rightEncoder,
            Encoder frontEncoder,
            AutonomousConfiguration configuration,
            Telemetry telemetry
    ) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.frontEncoder = frontEncoder;
        this.configuration = configuration;
        this.telemetry = telemetry;
    }

    public void update() {
        double leftPosition = leftEncoder.getCurrentPosition();
        double rightPosition = rightEncoder.getCurrentPosition();
        double frontPosition = frontEncoder.getCurrentPosition();

        double leftChange = leftPosition - prevLeftPosition;
        double rightChange = rightPosition - prevRightPosition;
        double angleChange = (leftChange - rightChange) / (configuration.getLateralDistance() * Utils.TICKS_PER_CM);

        double verticalChange = (leftChange + rightChange) / 2;
        double rawHorizontalChange = frontPosition - prevFrontPosition;
        double horizontalChange = rawHorizontalChange + (angleChange * configuration.getVerticalOffsetPerRadian());

        if (Math.abs(verticalChange) < 2000) verticalChange = 0;
        if (Math.abs(horizontalChange) < 2000) horizontalChange = 0;
        x += (horizontalChange * Math.cos(angle) -    verticalChange * Math.sin(angle)) / Utils.TICKS_PER_CM;
        y += (horizontalChange * Math.sin(angle) + verticalChange * Math.cos(angle)) / Utils.TICKS_PER_CM;
        angle += angleChange;

//        telemetry.addData("left", leftPosition);
//        telemetry.addData("right", rightPosition);
//        telemetry.addData("left change", leftChange);
//        telemetry.addData("right change", rightChange);
//        telemetry.addData("angle change", angleChange);
//        telemetry.addData("vertical change", verticalChange);
//        telemetry.addData("raw horizontal change", rawHorizontalChange);
//        telemetry.addData("horizontal change", horizontalChange);
//        telemetry.addData("x", x);
//        telemetry.addData("y", y);
//        telemetry.addData("angle", angle);
//        telemetry.addData("angle in degerees", angle * 180 / Math.PI);
//        telemetry.update();

        prevLeftPosition = leftPosition;
        prevRightPosition = rightPosition;
        prevFrontPosition = frontPosition;
    }

    @Override
    public void run() {
        while (isRunning) {
            update();
            try {
                Thread.sleep(configuration.getCoordinateDelay());
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void stop() {
        isRunning = false;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }
}
