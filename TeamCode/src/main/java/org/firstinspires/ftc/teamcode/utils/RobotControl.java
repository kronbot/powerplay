package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.KronBot;

// manages the movement of the robot
public class RobotControl {
    private final KronBot robot;

    // the acceleration power of the robot
    // used for a smooth rotation
    private final double accelerationPower = 0.05;

    // either when rotating or driving, we save the last power
    // if the robot starts rotating, the drive power becomes 0
    // if the robot starts moving, the rotate power becomes 0
    private double currentRotatePower = 0;
    private double currentDrivePower = 0;

    public RobotControl(KronBot robot) {
        this.robot = robot;
    }

    /**
     * Handles driving based on a given input.
     * The input should be betweeen [0, 1].
     * @param xInput the input on the x axis
     * @param yInput the input on the y axis
     */
    public void drive(double xInput, double yInput) {
        currentRotatePower = 0;
        double angle = Math.atan2(yInput, xInput);
        double power = Math.sqrt(xInput * xInput + yInput * yInput);

        if (power < Utils.EPS) // no power
            currentDrivePower = 0;
        else if (currentDrivePower < power) // smoother acceleration
            currentDrivePower = Math.min(currentDrivePower + accelerationPower, power);
        else // slowing down
            currentDrivePower = power;

        if (Utils.EPS < yInput) {
            if (angle < Math.PI / 2) { // moving to the right
                double wheelsDirection = Utils.map(angle, 0, 90, -1, 1);
                if (wheelsDirection < Utils.EPS)
                    wheelsDirection = 0;
                robot.drive(1, wheelsDirection, wheelsDirection, 1, currentDrivePower);
            } else { // moving to the left
                double wheelsDirection = Utils.map(angle, 90, 180, 1, -1);
                if (wheelsDirection < Utils.EPS)
                    wheelsDirection = 0;
                robot.drive(wheelsDirection, 1, 1, wheelsDirection, currentDrivePower);
            }
        } else if (yInput < -Utils.EPS) { // backwards
            if (angle < -90) { // moving to the left
                double wheelsDirection = Utils.map(angle, -180, -90, 1, -1);
                if (wheelsDirection > -Utils.EPS)
                    wheelsDirection = 0;
                robot.drive(-1, wheelsDirection, wheelsDirection, -1, currentDrivePower);
            } else { // moving to the right
                double wheelsDirection = Utils.map(angle, -90, 0, 0, 1);
                if (wheelsDirection > -Utils.EPS)
                    wheelsDirection = 0;
                robot.drive(wheelsDirection, -1, -1, wheelsDirection, currentDrivePower);
            }
        } else { // translate only (only moving on the x axis)
            if (-Utils.EPS < xInput && xInput < Utils.EPS) // no input
                robot.stopMotors();
            else if (xInput > Utils.EPS) // moving to the right
                robot.drive(1, -1, -1, 1, currentDrivePower);
            else // moving to the left
                robot.drive(-1, 1, 1, -1, currentDrivePower);
        }
    }

    /**
     * Rotates the robot using the given input.
     * The input should be betweeen [-1, 1].
     * @param direction if negative, rotating left, if positive, rotating right
     */
    public void rotate(double direction) {
        if (direction < Utils.EPS)
            return;

        currentRotatePower = 0;
        double power = 0;

        // starting the rotation smoothly
        if (currentRotatePower < Utils.EPS)
            currentRotatePower = 0.1;
        else // slow down
            currentRotatePower = Math.min(
                    currentRotatePower + accelerationPower,
                    Math.abs(direction)
            );

        // switching directions
        if (direction > 0)
            robot.drive(1, -1, 1, -1, currentRotatePower);
        else
            robot.drive(-1, 1, -1, 1, currentRotatePower);
    }

    /**
     * Translates the robot on the X-axis using the given input.
     * The input should be between [0, 1].
     * @param left the left direction value
     * @param right the right direction value
     */
    public void translate(double left, double right) {
        if (left < Utils.EPS || right < Utils.EPS)
            return;

        currentRotatePower = 0;
        double direction = right - left;

        // checking if direction too close to 0
        if (-Utils.EPS > direction && direction < Utils.EPS)
            return;

        // moving to the right
        if (direction > Utils.EPS)
            robot.drive(1, -1, -1, 1, direction);
        else // left
            robot.drive(-1, 1, 1, -1, Math.abs(direction));
    }
}
