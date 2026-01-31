package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HWMap;

@Autonomous(name="Real Far Auto RED")
public class farautored extends LinearOpMode {

    public HWMap robot = new HWMap();

    private static final double MOTOR_TICKS_PER_REV = 560.0;
    private static final double MOTOR_MAX_RPM = 300.0;
    private static final double SHOOTER_TO_MOTOR_RATIO = 4.0;

    public void encoderDrive ( double speed,
                               double fRightCounts, double fLeftCounts,
                               double bRightCounts, double bLeftCounts){

        int newfLeftTarget = robot.frontLeftDrive.getCurrentPosition();
        int newfRightTarget = robot.frontRightDrive.getCurrentPosition();
        int newbLeftTarget = robot.backLeftDrive.getCurrentPosition();
        int newbRightTarget = robot.backRightDrive.getCurrentPosition();

        robot.frontLeftDrive.setTargetPosition((int)newfLeftTarget + (int)fLeftCounts);
        robot.frontRightDrive.setTargetPosition((int)newfRightTarget + (int)fRightCounts);
        robot.backLeftDrive.setTargetPosition((int)newbLeftTarget + (int)bLeftCounts);
        robot.backRightDrive.setTargetPosition((int)newbRightTarget + (int)bRightCounts);

        double p = Math.abs(speed);
        robot.frontLeftDrive.setPower(p);
        robot.frontRightDrive.setPower(p);
        robot.backLeftDrive.setPower(p);
        robot.backRightDrive.setPower(p);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                robot.frontLeftDrive.isBusy() &&
                robot.frontRightDrive.isBusy() &&
                robot.backLeftDrive.isBusy() &&
                robot.backRightDrive.isBusy()) {
        }

        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        DcMotorEx flywheelOne = (DcMotorEx) robot.flywheelOne;
        DcMotorEx flywheelTwo = (DcMotorEx) robot.flywheelTwo;

        flywheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelOne.setVelocityPIDFCoefficients(10, 0.5, 0, 12);
        flywheelTwo.setVelocityPIDFCoefficients(10, 0.5, 0, 12);

        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Start", "FL:%d FR:%d BL:%d BR:%d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.backLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double shooterRPM = 340;

        double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
        motorRPM = Math.min(motorRPM, MOTOR_MAX_RPM);
        double targetTicksPerSec =
                motorRPM * MOTOR_TICKS_PER_REV / 60.0;

        flywheelOne.setVelocity(targetTicksPerSec);
        flywheelTwo.setVelocity(targetTicksPerSec);

        // ─────────────────────────────────────────────
        //      MIRRORED MOVEMENTS FOR RED
        // ─────────────────────────────────────────────

        // Straight back (same)
        encoderDrive(0.3, -200, -200, -200, -200);

        // Strafe/turn: FLIP LEFT-RIGHT signs
        encoderDrive(0.3, 250, -250, 250, -250);

        sleep(2000);

        robot.outtake.setPower(1);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(3000);
        robot.intake.setPower(0.8);
        sleep(250);
        robot.intake.setPower(0);
        robot.outtake.setPower(1);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(3000);
        robot.intake.setPower(-0.5);
        sleep(100);
        robot.intake.setPower(0.8);
        robot.outtake.setPower(1);
        sleep(300);
        robot.outtake.setPower(0);
        robot.intake.setPower(0);

        flywheelOne.setVelocity(0);
        flywheelTwo.setVelocity(0);

        // Straight (same)
        encoderDrive(0.5, -1000, -1000, -1000, -1000);

        // Turn: flip signs
        encoderDrive(0.5, -1150, 1150, -1150, 1150);

        robot.intake.setPower(0.9);
        // straight (same)
        encoderDrive(0.4, 1900, 1900, 1900, 1900);
        sleep(100);
        robot.intake.setPower(0);

        // straight (same)
        encoderDrive(0.5, -1550, -1550, -1550, -1550);

        flywheelOne.setVelocity(targetTicksPerSec);
        flywheelTwo.setVelocity(targetTicksPerSec);

        // turn: flip
        encoderDrive(0.5, 1110, -1110, 1110, -1110);

        // straight (same)
        encoderDrive(0.5, 1000, 1000, 1000, 1000);

        sleep(2000);

        robot.outtake.setPower(1);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(3000);
        robot.intake.setPower(0.8);
        sleep(250);
        robot.intake.setPower(0);
        robot.outtake.setPower(0.8);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(3000);
        robot.intake.setPower(-0.5);
        sleep(100);
        robot.intake.setPower(0.8);
        robot.outtake.setPower(0.8);
        sleep(300);
        robot.outtake.setPower(0);
        robot.intake.setPower(0);

        // straight (same)
        encoderDrive(0.5, -1000, -1000, -1000, -1000);

        flywheelOne.setVelocity(0);
        flywheelTwo.setVelocity(0);

        telemetry.addData("Done", "FL:%d FR:%d BL:%d BR:%d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.backLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();
    }
}
