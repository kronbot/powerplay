package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.Utils;

@Config
@Autonomous (name = "Odometry Calibration", group = "Calibrations")
public class OdometryCalibration extends LinearOpMode {
    // constants
    private final double ROTATE_SPEED = 0.5;

    private final KronBot robot = new KronBot();
    private BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);

        // giroscop cu butelie
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        telemetry.addData("Left encoder", robot.leftEncoder.getCurrentPosition());
        telemetry.addData("Right encoder", robot.rightEncoder.getCurrentPosition());
        telemetry.addData("Front encoder", robot.frontEncoder.getCurrentPosition());

        double angle = getAngle();
        telemetry.addData("Angle", angle);
        telemetry.update();

        waitForStart();

        // gettin' to 90 degrreeeeeeezzz
        // clockwiseee
        double target = 90, feedforwardStart = target * 2 / 3;
        while (angle < target && opModeIsActive()) {
            if (angle < feedforwardStart)
                robot.drive(1, -1, 1, -1, ROTATE_SPEED);
            else {
                double speed = Utils.map(target - angle, 0, target - feedforwardStart, 0.15, ROTATE_SPEED);
                robot.drive(1, -1, 1, -1, speed);
            }

            telemetry.addData("Angle", angle);
            telemetry.update();
            angle = getAngle();
        }

        robot.stopMotors();

        // meth go brrrrrr
        double horizontalEncoderDifference = Math.abs(
                Math.abs(robot.leftEncoder.getCurrentPosition()) +
                Math.abs(robot.rightEncoder.getCurrentPosition())
        );
        double verticalEncoderTicksPerDegree = horizontalEncoderDifference / angle;
        double lateralDistance = (verticalEncoderTicksPerDegree * 180) / (Math.PI * Utils.COUNTS_PER_CM);
        double ticksPerDegree = robot.frontEncoder.getCurrentPosition() / Math.toRadians(angle);

        // debug when not working :(
        while (opModeIsActive()) {
            telemetry.addData("Left", robot.leftEncoder.getCurrentPosition());
            telemetry.addData("Right", robot.rightEncoder.getCurrentPosition());
            telemetry.addData("Angle", angle);
            telemetry.addData("Vertical Encoder Ticks per degree", verticalEncoderTicksPerDegree);
            telemetry.addData("Lateral distance", lateralDistance);
            telemetry.addData("Ticks per degree", ticksPerDegree);
            telemetry.update();
        }
    }

    private double getAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }
}
