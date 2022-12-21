package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KronBot {
    public DcMotor frontLeftDc;
    public DcMotor frontRightDc;
    public DcMotor backLeftDc;
    public DcMotor backRightDc;

    public DcMotor slideDc;

    public Servo intakeServo;

    /**
     * Initialization of hardware map
     */
    public void initHardwareMap(HardwareMap hardwareMap) {
        frontLeftDc = hardwareMap.dcMotor.get("frontLeft");
        frontRightDc = hardwareMap.dcMotor.get("frontRight");
        backLeftDc = hardwareMap.dcMotor.get("backLeft");
        backRightDc = hardwareMap.dcMotor.get("backRight");

        frontLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);

        slideDc = hardwareMap.dcMotor.get("slide");
        slideDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeServo = hardwareMap.servo.get("intake");

        slideDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
     * Stops the motors from wheels
     */
    public void stopMotors() {
        frontLeftDc.setPower(0);
        frontRightDc.setPower(0);
        backLeftDc.setPower(0);
        backRightDc.setPower(0);
    }

    public void controlSlide(double power) {
        slideDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideDc.setPower(power);
    }

    public void resetSlideEncoder() {
        slideDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void controlIntake(double power) {
        intakeServo.setPosition(power);
    }

    public double intakePosition() {return intakeServo.getPosition();}
}
