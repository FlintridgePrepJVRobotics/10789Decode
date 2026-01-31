package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "2025Decode10789")
public class Teleop extends LinearOpMode {

    public HWMap robot = new HWMap();

    private static final double MOTOR_TICKS_PER_REV = 560.0;
    private static final double MOTOR_MAX_RPM = 300.0;
    private static final double SHOOTER_TO_MOTOR_RATIO = 4.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        DcMotorEx flywheelOne = robot.flywheelOne;
        DcMotorEx flywheelTwo = robot.flywheelTwo;

        flywheelOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        flywheelOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        double P = 10.0;
        double I = 0.5;
        double D = 0.0;
        double F = 12.0;

        flywheelOne.setVelocityPIDFCoefficients(P, I, D, F);
        flywheelTwo.setVelocityPIDFCoefficients(P, I, D, F);

        double driveSpeed = 1.0;

        double shooterRPM = 340;
        double minShooterRPM = 100;
        double maxShooterRPM = 1200;

        boolean toggleStateFlywheel = false;
        boolean wasPressedFlywheel = false;

        boolean toggleStateIntake = false;
        boolean wasPressedIntake = false;
        boolean wasPressedB = false;
        boolean wasPressedA = false;

        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        waitForStart();

        while (opModeIsActive()) {

            // ----------------- INTAKE -----------------
            if (gamepad1.y) {
                robot.outtake.setPower(1);
            } else {
                if (gamepad1.a) {
                    robot.intake.setPower(-0.8);
                    robot.outtake.setPower(-1);
                } else {
                    if (gamepad1.b) {
                        robot.intake.setPower(0.8);
                        robot.outtake.setPower(1);
                    } else {
                        if (gamepad1.left_bumper && !wasPressedIntake) {
                            toggleStateIntake = !toggleStateIntake;
                        }
                        wasPressedIntake = gamepad1.left_bumper;

                        if (toggleStateIntake) {
                            robot.intake.setPower(0.8);
                            robot.outtake.setPower(0.4);
                        } else {
                            robot.intake.setPower(0);
                            robot.outtake.setPower(0);
                        }
                    }
                }
            }

            // ----------------- DRIVETRAIN -----------------
            double y = gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x * 1.1;
            double rx = -gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            robot.frontLeftDrive.setPower(frontLeftPower * driveSpeed);
            robot.backLeftDrive.setPower(backLeftPower * driveSpeed);
            robot.frontRightDrive.setPower(frontRightPower * driveSpeed);
            robot.backRightDrive.setPower(backRightPower * driveSpeed);

            // ----------------- SHOOTER RPM ADJUST -----------------
            if (gamepad1.dpad_up && !prevDpadUp) {
                shooterRPM += 10;
                shooterRPM = Math.min(shooterRPM, maxShooterRPM);
            }

            if (gamepad1.dpad_down && !prevDpadDown) {
                shooterRPM -= 10;
                shooterRPM = Math.max(shooterRPM, minShooterRPM);
            }

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // ----------------- FLYWHEEL TOGGLE -----------------
            if (gamepad1.right_bumper && !wasPressedFlywheel) {
                toggleStateFlywheel = !toggleStateFlywheel;
            }
            wasPressedFlywheel = gamepad1.right_bumper;

            double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
            motorRPM = Math.min(motorRPM, MOTOR_MAX_RPM);

            double targetTicksPerSec = motorRPM * MOTOR_TICKS_PER_REV / 60.0;

            if (toggleStateFlywheel) {
                flywheelOne.setVelocity(targetTicksPerSec);
                flywheelTwo.setVelocity(targetTicksPerSec);
            } else {
                flywheelOne.setVelocity(0);
                flywheelTwo.setVelocity(0);
            }

            // ----------------- TELEMETRY -----------------
            telemetry.addData("Intake Toggle", toggleStateIntake ? "ON" : "OFF");
            telemetry.addData("Flywheel Toggle", toggleStateFlywheel ? "ON" : "OFF");

            telemetry.addData("Target Shooter RPM", shooterRPM);
            telemetry.addData("Target Motor RPM", motorRPM);
            telemetry.addData("Target TPS", targetTicksPerSec);

            telemetry.addData("Fly1 TPS", flywheelOne.getVelocity());
            telemetry.addData("Fly2 TPS", flywheelTwo.getVelocity());

            telemetry.update();
        }
    }
}
