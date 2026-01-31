package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HWMap;

@Autonomous(name="Real Close Auto RED")
public class closeautored extends LinearOpMode {

    public HWMap robot = new HWMap();

    // ===== SHOOTER CONSTANTS (same as TeleOp) =====
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

        while (robot.backRightDrive.isBusy() &&
                robot.backLeftDrive.isBusy() &&
                robot.frontRightDrive.isBusy() &&
                robot.frontLeftDrive.isBusy()) {
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

        // Initialize hardware
        robot.init(hardwareMap);

        DcMotorEx flywheelOne = (DcMotorEx) robot.flywheelOne;
        DcMotorEx flywheelTwo = (DcMotorEx) robot.flywheelTwo;

        // Shooter encoder setup
        flywheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Same PIDF as TeleOp
        flywheelOne.setVelocityPIDFCoefficients(10, 0.5, 0, 12);
        flywheelTwo.setVelocityPIDFCoefficients(10, 0.5, 0, 12);

        // Reset encoders
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

        double shooterRPM = 290;

        double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
        motorRPM = Math.min(motorRPM, MOTOR_MAX_RPM);
        double targetTicksPerSec =
                motorRPM * MOTOR_TICKS_PER_REV / 60.0;

        flywheelOne.setVelocity(targetTicksPerSec);
        flywheelTwo.setVelocity(targetTicksPerSec);

        // ===== SAME straight drive =====
        encoderDrive(0.8, 2000, 2000, 2000, 2000);

        sleep(3000);

        // ===== SHOOT 3 =====
        robot.outtake.setPower(1);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(2500);
        robot.intake.setPower(0.8);
        sleep(250);
        robot.intake.setPower(0);
        robot.outtake.setPower(0.8);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(2500);
        robot.intake.setPower(-0.5);
        sleep(100);
        robot.intake.setPower(0.8);
        robot.outtake.setPower(1);
        sleep(300);
        robot.outtake.setPower(0);
        robot.intake.setPower(0);

        // ===== MIRROR TURN/STRAFE (FLIP SIGNS) =====
        // Blue: encoderDrive(0.7, 540, -540, 540, -540);
        encoderDrive(0.7, -540, 540, -540, 540);

        encoderDrive(0.8, 400, 400, 400, 400); // straight (same)

        // Blue: encoderDrive(0.8, 1000, -1000, 1000, -1000);
        encoderDrive(0.8, -1000, 1000, -1000, 1000);

        robot.intake.setPower(0.9);
        encoderDrive(0.4, 2000, 2000, 2000, 2000);
        sleep(200);
        robot.intake.setPower(0);

        // ===== MIRROR BACKWARDS TURN =====
        // Blue: encoderDrive(0.8, -1700,-1700,-1700,-1700); (straight)
        encoderDrive(0.8, -1700, -1700, -1700, -1700);

        // Blue: encoderDrive(0.6,-1400,1400,-1400,1400);
        encoderDrive(0.6, 1400, -1400, 1400, -1400);

        // ===== SHOOT AGAIN =====
        flywheelOne.setVelocity(targetTicksPerSec);
        flywheelTwo.setVelocity(targetTicksPerSec);

        sleep(3000);

        robot.outtake.setPower(1);
        sleep(100);
        robot.outtake.setPower(0);

        sleep(2500);
        robot.intake.setPower(0.8);
        sleep(250);
        robot.intake.setPower(0);
        robot.outtake.setPower(0.8);
        sleep(300);
        robot.outtake.setPower(0);

        sleep(2500);
        robot.intake.setPower(-0.5);
        sleep(100);
        robot.intake.setPower(0.8);
        robot.outtake.setPower(0.8);
        sleep(300);
        robot.outtake.setPower(0);
        robot.intake.setPower(0);

        // ===== MIRROR END TURN/STRAFE =====
        // Blue: encoderDrive(1,700,-700,700,-700);
        encoderDrive(1, -700, 700, -700, 700);

        encoderDrive(1, 1200, 1200, 1200, 1200); // straight

        telemetry.addData("Done", "Finished RED Close Auto");
        telemetry.update();
    }
}
