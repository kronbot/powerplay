package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.Utils;

@Autonomous
public class OdometryCalibration extends LinearOpMode {
    // constants
    private final double ROTATE_SPEED = 0.69;

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

        waitForStart();

        // gettin' to 90 degrreeeeeeezzz
        // clockwiseee
        double angle = getAngle();
        while (angle < 90 && opModeIsActive()) {
            if (angle < 60)
                robot.drive(1, -1, 1, -1, ROTATE_SPEED);
            else {
                // derviate of f(x) = x ^ 2
                // df / dx = 2x
                double speed = 1 - 2 * Utils.map(angle, 0, 90, 0, 1);
                robot.drive(1, -1, 1, -1, speed);
            }

            telemetry.addData("Angle", angle);
            telemetry.update();
            angle = getAngle();
        }
    }

    private double getAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }
}
