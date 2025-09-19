package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        lastTime = System.nanoTime();
    }

    public double update(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9; // seconds
        lastTime = currentTime;

        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }
}