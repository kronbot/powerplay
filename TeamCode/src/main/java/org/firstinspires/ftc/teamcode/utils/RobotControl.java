package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.KronBot;
import com.qualcomm.robotcore.util.Util;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// manages the movement of the robot
public class RobotControl {
    private final KronBot robot;
    private final Telemetry telemetry;

    // the acceleration power of the robot
    // used for a smooth rotation
    private final double accelerationPower = 0.05;

    // either when rotating or driving, we save the last power
    // if the robot starts rotating, the drive power becomes 0
    // if the robot starts moving, the rotate power becomes 0
    private double currentRotatePower = 0;
    private double currentDrivePower = 0;

    public RobotControl(KronBot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    /**
     * Handles driving based on a given input.
     * The input should be betweeen [-1, 1].
     * @param xInput the input on the x axis
     * @param yInput the input on the y axis
     * @returns true if the robot moves, false if not
     */
    public boolean drive(double xInput, double yInput) {
        if (
                (-Utils.EPS < xInput && xInput < Utils.EPS) &&
                        (-Utils.EPS < yInput && yInput < Utils.EPS)
        )
            return false;

        currentRotatePower = 0;
        double angle = Math.atan2(yInput, xInput);
        double power = Math.sqrt(xInput * xInput + yInput * yInput);

        if (power < Utils.EPS) // no power
            currentDrivePower = 0;
        else if (currentDrivePower < power) // smoother acceleration
            currentDrivePower = Math.min(currentDrivePower + accelerationPower, power);
        else // slowing down
            currentDrivePower = power;

        // both or only Y-axis
        if (Utils.EPS < yInput) {
            if (angle > 0 && angle < Math.PI / 2) { // moving to the right
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
            return true;
        } else if (yInput < -Utils.EPS) { // backwards
            if (angle > -(Math.PI / 2) && angle < 0) { // moving to the left
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
            return true;
        }

        // translate only on the X axis (only moving on the x axis)
        if (xInput > Utils.EPS) // moving to the right
            robot.drive(1, -1, -1, 1, currentDrivePower);
        else // moving to the left
            robot.drive(-1, 1, 1, -1, currentDrivePower);
        return true;
    }

    /**
     * Rotates the robot using the given input.
     * The input should be betweeen [-1, 1].
     * @param direction if negative, rotating left, if positive, rotating right
     * @returns true if the robot rotates, false if the direction is 0
     */
    public boolean rotate(double direction) {
        if (-Utils.EPS < direction && direction < Utils.EPS)
            return false;
        currentDrivePower = 0;

        // starting the rotation smoothly
        double power = Math.abs(direction);
        // smooth start
        if (currentRotatePower < Utils.EPS)
            currentRotatePower = Utils.EPS;
        else if (power > currentRotatePower)
            currentRotatePower = Math.min(currentRotatePower + accelerationPower, power);
        else // slowing down
            currentRotatePower = power;

        // switching directions
        if (direction > 0)
            robot.drive(1, -1, 1, -1, currentRotatePower);
        else
            robot.drive(-1, 1, -1, 1, currentRotatePower);
        return true;
    }

    /**
     * Translates the robot on the X-axis using the given input.
     * The input should be between [0, 1].
     * @param left the left direction value
     * @param right the right direction value
     * @returns true if the robot translates, false if not
     */
    public boolean translate(double left, double right) {
        double direction = right - left;
        if (-Utils.EPS < direction && direction < Utils.EPS)
            return false;
        currentRotatePower = 0;
        // moving to the right
        if (direction > Utils.EPS)
            robot.drive(1, -1, -1, 1, direction);
        else // left
            robot.drive(-1, 1, 1, -1, Math.abs(direction));;

        return true;
    }
}
