package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.OdometryGlobalCoordinatePositionAdaptat;

import java.util.ArrayList;
import java.util.List;

//@Autonomous(name = "Odometrie")
public class Odometrie extends LinearOpMode {

    KronBot robot = new KronBot();
    ElapsedTime timer = new ElapsedTime();
    //Odometry Wheels
    //DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 685; // constanta calculata. daca da pe langa modif aici
    final double kp = 0.01, kd = 0.005;
    final double kpDistance = 0.05, kpRotation = 0.01;
    final double allowableErrorRotation = 5,  allowableErrorForward = 3;

    double error_old;

    OdometryGlobalCoordinatePositionAdaptat globalPositionUpdate;

    List<Double> powerMotors = new ArrayList<Double>(); // puterile pentru fiecare dintre cele 4 motoare

    DcMotor orizontal;

    private static final double[] distancePositions = {7,10,26};
    private static final double[] armPositions = {0.17,0.31,0.47};
    private static final double[] cupaPositions = {0.18,0.33,1};

    @Override
    public void runOpMode() throws InterruptedException {
        initDriveHardwareMap();

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePositionAdaptat(robot.frontLeftDc, robot.frontRightDc, orizontal, COUNTS_PER_INCH, 0);
        globalPositionUpdate.reverseNormalEncoder();

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        /*//goToPosition(-50,0.5,globalPositionUpdate.returnOrientation(),3);

        goToPosition(-97,0.5,0,allowableErrorForward);
        //goForward(-93, 0.8, 3);
        if(!opModeIsActive())
            return;

        rotateToDegree(90, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        if(!opModeIsActive())
            return;

        rotateToDegree(27, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        goToPosition(102, 0.5, 27, allowableErrorForward);
        if(!opModeIsActive())
            return;

        rotateToDegree(-62, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        robot.drive(1,-1,-1,1,0.2);
        wait_seconds(0.8);
        if(!opModeIsActive())
            return;
        robot.stopMotors();

        robot.squishy.setPower(-0.8);
        wait_seconds(4);
        robot.squishy.setPower(0);
        if(!opModeIsActive())
            return;

        rotateToDegree(0, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        goToPosition(-59,0.5,0,allowableErrorForward);*/


        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.frontLeftDc.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.frontRightDc.getCurrentPosition());
            telemetry.addData("horizontal encoder position", orizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap() {

        robot.frontLeftDc = hardwareMap.dcMotor.get("frontLeft");
        robot.frontRightDc = hardwareMap.dcMotor.get("frontRight");
        robot.backLeftDc = hardwareMap.dcMotor.get("backLeft");
        robot.backRightDc = hardwareMap.dcMotor.get("backRight");
        robot.frontLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.frontLeftDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeftDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }


    double GetMinErrorAngle(double currOrientation, double desireOrientation) {

        double beta = (360 - currOrientation + desireOrientation) % 360;
        double gama = beta - 360;

        if (Math.abs(beta) < Math.abs(gama))
            return beta;
        else return gama;

    }

    double PIDAngle(double desire, double act){

        double error_act = GetMinErrorAngle(desire, act);
        double valToReturn = error_act * kp + (error_act - error_old) * kd;
        error_old = error_act;
        return valToReturn;
    }

    double PIDistance(double distance) {
        double valToReturn = distance * kpDistance;
        return valToReturn;
    }

    double PIDRotation(double rotation) {
        double valToReturn = rotation * kpRotation;
        return valToReturn;
    }

    public void goForward(double targetYPosition, double robotPower, double allowableDistanceError) {
        globalPositionUpdate.resetEncoders();

        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());
        error_old=0;

        while (opModeIsActive() && distanceToYTarget > allowableDistanceError) {
            distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

            double pidistance = PIDistance(distanceToYTarget / COUNTS_PER_INCH);

            if (pidistance > 1)
                pidistance = 1;

            pidistance = Map(pidistance, 0, 1, 0, robotPower);

            pidistance = newSpeed(pidistance,robotPower);

            if (targetYPosition < globalPositionUpdate.returnYCoordinate())
                pidistance *= -1;

            if(pidistance==0)
                break;

            robot.driveWithSpeeds(pidistance, pidistance, pidistance, pidistance);

            telemetry.addData("Distance to Target:", distanceToYTarget / COUNTS_PER_INCH);
            telemetry.addData("allowable distance error: ", allowableDistanceError / COUNTS_PER_INCH);
            telemetry.addData("coord x:", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("coord y:", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.update();
        }

        telemetry.addData("Distance to target: ", "Target Reached");
        telemetry.update();
        robot.stopMotors();
    }

    public void goToPosition(double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError)
    {
        globalPositionUpdate.resetEncoders();

        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

        while (opModeIsActive() && distanceToYTarget > allowableDistanceError) {
            distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

            double pidangle = PIDAngle(desiredRobotOrientation, globalPositionUpdate.returnOrientation());
            double pidistance = PIDistance(distanceToYTarget / COUNTS_PER_INCH);

            telemetry.addData("PidAngle", pidangle);

            if(pidistance > 1)
                pidistance = 1;
            pidistance = Map(pidistance, 0, 1, 0, robotPower);
            //telemetry.addData("Pidistance dupa map", pidistance);

            if (targetYPosition < globalPositionUpdate.returnYCoordinate())
                pidistance *= -1;

            double leftMotorSpeed = pidistance - pidangle;
            double rightMotorSpeed = pidistance + pidangle;

            leftMotorSpeed = newSpeed(leftMotorSpeed, robotPower);
            rightMotorSpeed = newSpeed(rightMotorSpeed, robotPower);

            robot.driveWithSpeeds(leftMotorSpeed, rightMotorSpeed, leftMotorSpeed, rightMotorSpeed);

            telemetry.addData("Left power:", leftMotorSpeed);
            telemetry.addData("Right power:", rightMotorSpeed);
            telemetry.addData("Distance to Target:", distanceToYTarget/COUNTS_PER_INCH);
            telemetry.addData("allowable distance error: ", allowableDistanceError/COUNTS_PER_INCH);
            telemetry.addData("coord x:", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("coord y:", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.update();
        }

        telemetry.addData("Distance to target: ", "Target Reached");
        telemetry.update();
        robot.stopMotors();
    }

    public void rotateToDegree(double targetDegree, double allowableAngleError, double robotRotationPower) {
        double currentOrientation = globalPositionUpdate.returnOrientation();
        telemetry.addData("valoare", Math.abs(targetDegree - currentOrientation));

        while (opModeIsActive() && Math.abs(targetDegree - currentOrientation) > allowableAngleError) {
            double pidRotation = PIDRotation(GetMinErrorAngle(currentOrientation, targetDegree));
            telemetry.addData("current Orientation:", currentOrientation);
            telemetry.addData("pidRotation", pidRotation);
            double leftMotorSpeed = pidRotation;
            double rightMotorSpeed = -pidRotation;

            leftMotorSpeed = newSpeed(leftMotorSpeed, robotRotationPower);
            rightMotorSpeed = newSpeed(rightMotorSpeed, robotRotationPower);
            if(leftMotorSpeed == 0 || rightMotorSpeed == 0)
                break;
            telemetry.addData("leftSpeed", leftMotorSpeed);
            telemetry.addData("rightSpeed", rightMotorSpeed);
            telemetry.update();
            currentOrientation = globalPositionUpdate.returnOrientation();
            robot.driveWithSpeeds(leftMotorSpeed, rightMotorSpeed, leftMotorSpeed, rightMotorSpeed);

        }
        robot.drive(1, 1, 1, 1, 0);

    }

    double newSpeed(double vit, double robotSpeed) {
        if (vit > robotSpeed)
            return robotSpeed;
        if (Math.abs(vit) < 0.1)
            return 0;
        if (vit < -robotSpeed)
            return -robotSpeed;
        return vit;
    }

    double Map(double x, double in_min, double in_max, double out_min, double out_max) {

        double rez = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return rez;

    }

    void wait_seconds(double seconds){
        timer.reset();
        while (timer.seconds() < seconds && opModeIsActive()) ;
    }
}
