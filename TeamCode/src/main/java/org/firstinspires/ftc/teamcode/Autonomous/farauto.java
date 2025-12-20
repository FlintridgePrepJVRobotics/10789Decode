package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HWMap;

@Autonomous(name = "FarAutoBlue")
public class farauto extends LinearOpMode {

    public HWMap robot = new HWMap();

    // ====== SHOOTER CONSTANTS (match your TeleOp) ======
    private static final double MOTOR_TICKS_PER_REV = 560.0; // change if your motor is different
    private static final double MOTOR_MAX_RPM = 300.0;       // change if your motor is different
    private static final double SHOOTER_TO_MOTOR_RATIO = 4.0; // change to your real ratio

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Cast flywheels to DcMotorEx so we can use setVelocity()
        DcMotorEx flywheelOne = (DcMotorEx) robot.flywheelOne;
        DcMotorEx flywheelTwo = (DcMotorEx) robot.flywheelTwo;

        // Encoder + velocity mode setup (like TeleOp)
        flywheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // PIDF (use the same starter values as TeleOp)
        double P = 10.0, I = 0.5, D = 0.0, F = 12.0;
        flywheelOne.setVelocityPIDFCoefficients(P, I, D, F);
        flywheelTwo.setVelocityPIDFCoefficients(P, I, D, F);

        waitForStart();
        if (isStopRequested()) return;

        driveForward(2000, 0.5);
        turnLeft(500, 0.4);

        double shooterRPM = 360;
        setShooterRPM(flywheelOne, flywheelTwo, shooterRPM);
        sleep(1200);

        robot.feedServo.setPosition(0);
        sleep(1050);
        robot.feedServo.setPosition(1);
        flywheelOne.setPower(0);
        flywheelTwo.setPower(0);


        robot.intake.setPower(1);
        sleep(300);
        robot.intake.setPower(0);

        setShooterRPM(flywheelOne, flywheelTwo, shooterRPM);
        sleep(1200);
        robot.feedServo.setPosition(0);
        sleep(1050);
        robot.feedServo.setPosition(1);
        flywheelOne.setPower(0);
        flywheelTwo.setPower(0);

        robot.intake.setPower(1);
        sleep(300);
        robot.intake.setPower(0);

        setShooterRPM(flywheelOne, flywheelTwo, shooterRPM);
        sleep(1200);
        robot.feedServo.setPosition(0);
        sleep(1050);
        robot.feedServo.setPosition(1);
        flywheelOne.setPower(0);
        flywheelTwo.setPower(0);

        stopAllDrive();
    }

    // ====== Shooter helper ======
    private void setShooterRPM(DcMotorEx m1, DcMotorEx m2, double shooterRPM) {
        // Convert flywheelRPM -> motorRPM based on ratio
        double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
        if (motorRPM > MOTOR_MAX_RPM) motorRPM = MOTOR_MAX_RPM;

        // Convert motorRPM -> ticks/sec
        double targetTicksPerSec = motorRPM * MOTOR_TICKS_PER_REV / 60.0;

        m1.setVelocity(targetTicksPerSec);
        m2.setVelocity(targetTicksPerSec);

        telemetry.addData("Target Shooter RPM", shooterRPM);
        telemetry.addData("Target motorRPM", motorRPM);
        telemetry.addData("Target ticks/s", targetTicksPerSec);
        telemetry.update();
    }

    // ====== Drive helpers (time-based) ======
    private void driveForward(int timeMs, double speed) {
        robot.frontLeftDrive.setPower(-speed);
        robot.frontRightDrive.setPower(-speed);
        robot.backLeftDrive.setPower(-speed);
        robot.backRightDrive.setPower(-speed);
        sleep(timeMs);
        stopAllDrive();
    }

    private void turnLeft(int timeMs, double speed) {
        robot.frontLeftDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(-speed);
        robot.backRightDrive.setPower(-speed);
        sleep(timeMs);
        stopAllDrive();
    }

    private void stopAllDrive() {
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
    }
}
