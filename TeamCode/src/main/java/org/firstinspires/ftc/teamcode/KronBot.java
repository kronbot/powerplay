package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KronBot {
    HardwareMap hardwareMap;

    DcMotor frontLeftDc;
    DcMotor frontRightDc;
    DcMotor backLeftDc;
    DcMotor backRightDc;

    /**
     * Initialization of hardware map
     */
    public void initHardwareMap() {
        frontLeftDc = hardwareMap.dcMotor.get("frontLeft");
        frontRightDc = hardwareMap.dcMotor.get("frontRight");
        backLeftDc = hardwareMap.dcMotor.get("backLeft");
        backRightDc = hardwareMap.dcMotor.get("backRight");

        frontLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Set the power of the motors from wheels
     *
     * @param frontLeft  direction of the front left wheel -1 or 1
     * @param frontRight direction of the front right wheel -1 or 1
     * @param backLeft   direction of the back left wheel -1 or 1
     * @param backRight  direction of the back right wheel -1 or 1
     * @param power      the power to give to all four wheels [0,1]
     */
    public void drive(double frontLeft, double frontRight, double backLeft, double backRight, double power) {
        frontLeftDc.setPower(frontLeft * power);
        frontRightDc.setPower(frontRight * power);
        backLeftDc.setPower(backLeft * power);
        backRightDc.setPower(backRight * power);
    }

    /**
     * Set the power of each individual motor from wheels
     *
     * @param powerFrontLeft  the power to give to front left wheel [0,1]
     * @param powerFrontRight the power to give to front right wheel [0,1]
     * @param powerBackLeft   the power to give to back left wheel [0,1]
     * @param powerBackRight  the power to give to back right wheel [0,1]
     */
    public void driveWithSpeeds(double powerFrontLeft, double powerFrontRight, double powerBackLeft, double powerBackRight){
        frontLeftDc.setPower(powerFrontLeft);
        frontRightDc.setPower(powerFrontRight);
        backLeftDc.setPower(powerBackLeft);
        backRightDc.setPower(powerBackRight);
    }

    /**
     * Stops the motors from wheels
     */
    public void stopMotors() {
        frontLeftDc.setPower(0);
        frontRightDc.setPower(0);
        backLeftDc.setPower(0);
        backRightDc.setPower(0);
    }

}
