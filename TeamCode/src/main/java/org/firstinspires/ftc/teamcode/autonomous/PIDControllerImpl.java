package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControllerImpl implements PIDController {
    private final double kp, ki, kd;
    private double target = 0, integralSum = 0, lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDControllerImpl(AutonomousConfiguration configuration) {
        this.kp = configuration.getKp();
        this.ki = configuration.getKi();
        this.kd = configuration.getKd();
    }

    @Override
    public double getSpeed(double current) {
        double error = target - current;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        lastError = error;
        timer.reset();
        return (kp * error) + (ki * integralSum) + (kd * derivative);
    }

    @Override
    public double getTarget() {
        return target;
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        this.integralSum = 0;
        this.lastError = 0;
        this.timer.reset();
    }
}
