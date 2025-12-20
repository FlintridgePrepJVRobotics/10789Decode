package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "2025Decode10789")
public class Teleop extends LinearOpMode {

    public HWMap robot = new HWMap();

    // === CONSTANTS YOU SHOULD TUNE ===
    // REV-41-1291 (HD Hex 20:1): 560 ticks per rev, ~300 RPM max
    private static final double MOTOR_TICKS_PER_REV = 560.0;
    private static final double MOTOR_MAX_RPM      = 300.0;

    // Shooter gear ratio: flywheelRPM = motorRPM * SHOOTER_TO_MOTOR_RATIO
    // Example: 1:4 ratio (small motor pulley, big flywheel pulley) → 4.0
    private static final double SHOOTER_TO_MOTOR_RATIO = 4.0; // CHANGE THIS to your real ratio

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // --- CAST FLYWHEELS TO DcMotorEx ---
        DcMotorEx flywheelOne = (DcMotorEx) robot.flywheelOne;
        DcMotorEx flywheelTwo = (DcMotorEx) robot.flywheelTwo;

        // Reset and set encoder mode
        flywheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // PIDF – starting values, you should tune later
        double P = 10.0;
        double I = 0.5;
        double D = 0.0;
        double F = 12.0;

        flywheelOne.setVelocityPIDFCoefficients(P, I, D, F);
        flywheelTwo.setVelocityPIDFCoefficients(P, I, D, F);

        // Drive speed
        double driveSpeed = 1.0;

        // Shooter speed in *flywheel RPM* (not motor RPM)
        // Adjust this range to what feels good for your shooter
        double shooterRPM   = 370;  // starting flywheel RPM
        double minShooterRPM = 100; // minimum flywheel RPM
        double maxShooterRPM = 1200; // maximum flywheel RPM

        // Toggle states

        boolean toggleStateFlywheel = false;
        boolean wasPressedFlywheel = false;

        boolean toggleStateIntake = false;
        boolean wasPressedIntake = false;

        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        waitForStart();

        while (opModeIsActive()) {

            // ----------------- DRIVETRAIN -----------------
            double y = gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = -gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeftDrive.setPower(frontLeftPower * driveSpeed);
            robot.backLeftDrive.setPower(backLeftPower * driveSpeed);
            robot.frontRightDrive.setPower(frontRightPower * driveSpeed);
            robot.backRightDrive.setPower(backRightPower * driveSpeed);

            // ----------------- SHOOTER SPEED ADJUST (D-PAD) -----------------
            // Dpad up/down change the *flywheel* RPM (shooterRPM)
            if (gamepad1.dpad_up && !prevDpadUp) {
                shooterRPM += 10; // step size in flywheel RPM
                if (shooterRPM > maxShooterRPM) shooterRPM = maxShooterRPM;
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                shooterRPM -= 10;
                if (shooterRPM < minShooterRPM) shooterRPM = minShooterRPM;
            }
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // ----------------- INTAKE (HOLD + REVERSE) -----------------
// Hold LEFT BUMPER to intake
// Hold RIGHT TRIGGER to reverse intake

            if (gamepad1.left_bumper) {
                robot.intake.setPower(-1);   // intake in
            }
            else if (gamepad1.right_trigger > 0.2) {
                robot.intake.setPower(1);    // intake reverse
            }
            else {
                robot.intake.setPower(0);    // stop intake
            }

//            // ----------------- INTAKE TOGGLE -----------------
//            if (gamepad1.left_bumper && !wasPressedIntake) {
//                toggleStateIntake = !toggleStateIntake;
//            }
//            wasPressedIntake = gamepad1.left_bumper;
//
//            if (toggleStateIntake) {
//                robot.intake.setPower(-1);
//            } else {
//                robot.intake.setPower(0);
//            }

            // ----------------- FLYWHEEL TOGGLE + VELOCITY CONTROL -----------------
            if (gamepad1.right_bumper && !wasPressedFlywheel) {
                toggleStateFlywheel = !toggleStateFlywheel;
            }
            wasPressedFlywheel = gamepad1.right_bumper;

            // Convert shooterRPM (flywheel) → motorRPM using gear ratio
            double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
            if (motorRPM > MOTOR_MAX_RPM) {
                motorRPM = MOTOR_MAX_RPM; // clamp to motor capability
            }

            // Convert motorRPM → ticks per second
            double targetTicksPerSec = motorRPM * MOTOR_TICKS_PER_REV / 60.0;

            if (toggleStateFlywheel) {
                flywheelOne.setVelocity(targetTicksPerSec);
                flywheelTwo.setVelocity(targetTicksPerSec);
            } else {
                flywheelOne.setVelocity(0);
                flywheelTwo.setVelocity(0);
            }

            // ----------------- FEED SERVO -----------------
            if (gamepad1.x) {
                robot.feedServo.setPosition(0);
            } else {
                robot.feedServo.setPosition(1);
            }

            // ----------------- TELEMETRY -----------------
            double fly1_tps = flywheelOne.getVelocity();
            double fly2_tps = flywheelTwo.getVelocity();

            double fly1_motorRPM = fly1_tps * 60.0 / MOTOR_TICKS_PER_REV;
            double fly2_motorRPM = fly2_tps * 60.0 / MOTOR_TICKS_PER_REV;

            double fly1_shooterRPM = fly1_motorRPM * SHOOTER_TO_MOTOR_RATIO;
            double fly2_shooterRPM = fly2_motorRPM * SHOOTER_TO_MOTOR_RATIO;

            telemetry.addData("Target Shooter RPM", shooterRPM);
            telemetry.addData("Target Motor RPM", motorRPM);
            telemetry.addData("Target tps", targetTicksPerSec);

            telemetry.addData("Motor1 RPM", fly1_motorRPM);
            telemetry.addData("Motor2 RPM", fly2_motorRPM);
            telemetry.addData("Shooter1 RPM (est)", fly1_shooterRPM);
            telemetry.addData("Shooter2 RPM (est)", fly2_shooterRPM);

            telemetry.addData("Fly1 tps", fly1_tps);
            telemetry.addData("Fly2 tps", fly2_tps);
            telemetry.addData("Fly1 pos", flywheelOne.getCurrentPosition());
            telemetry.addData("Fly2 pos", flywheelTwo.getCurrentPosition());
            telemetry.update();
        }
    }
}
//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "2025Decode10789")
//public class Teleop extends LinearOpMode {
//    public HWMap robot = new HWMap();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        waitForStart();
//        double speed = 1;
//
//        boolean previousGamepad1_dpad_up = false;
//        boolean previousGamepad1_dpad_down = false;
//
//        int ticksPerRev =28;
//        double flywheelSpeed = 20;
//
//        double P = 10.0;
//        double I = 3.0;
//        double D = 0.0;
//        double F = 12.0;
//
//
//        boolean toggleStateFlywheel = false; // The variab le that holds the toggled state (e.g., claw open/closed)
//        boolean wasPressedFlywheel = false;  // The variable to store the button's state from the previous cycle
//        boolean toggleStateIntake = false; // The variable that holds the toggled state (e.g., claw open/closed)
//        boolean wasPressedIntake = false;  // The variable to store the button's state from the previous cycle
//
//        int checkInterval = 200;
//        int prevPositionnew1 = robot.flywheelOne.getCurrentPosition();
//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        timer.reset();
//
//        int prevPositionnew2 = robot.flywheelTwo.getCurrentPosition();
//        ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        timer1.reset();
//
//        while (opModeIsActive()) {
//
//            boolean currentGamepad1_dpad_up = gamepad1.dpad_up;
//            boolean currentGamepad1_dpad_down = gamepad1.dpad_down;
//
//            double y = -gamepad1.right_stick_y;
//            double x = gamepad1.right_stick_x * 1.1;
//            double rx = -gamepad1.left_stick_x;
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y - x + rx) / denominator;
//            double backLeftPower = (y + x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            robot.frontLeftDrive.setPower(frontLeftPower * speed);
//            robot.backLeftDrive.setPower(backLeftPower * speed);
//            robot.frontRightDrive.setPower(frontRightPower * speed);
//            robot.backRightDrive.setPower(backRightPower * speed);
//
//
//            telemetry.addData("flywheel speed", flywheelSpeed);
//
//
////            if (timer.time() > checkInterval) {
////                    double speednew1 = (double) (robot.flywheelOne.getCurrentPosition() - prevPositionnew1) / timer.time();
////
////                    // This will print out the ticks per millisecond of the motor.
////                    telemetry.addData("Ticks per ms for motor 1", speednew1);
////                    telemetry.update();
////                    prevPositionnew1 = robot.flywheelOne.getCurrentPosition();
////                    timer.reset();
////
////            }
//
////            if (timer1.time() > checkInterval) {
////                    double speednew2 = (double) (robot.flywheelTwo.getCurrentPosition() - prevPositionnew2) / timer1.time();
////
////                    // This will print out the ticks per millisecond of the motor.
////                    telemetry.addData("Ticks per ms for motor 2", speednew2);
////                    telemetry.update();
////                    prevPositionnew2 = robot.flywheelTwo.getCurrentPosition();
////                    timer.reset();
////
////            }
//
//            if (gamepad1.dpad_up){
//                flywheelSpeed = flywheelSpeed + 5;
//                sleep(250);
//
//            }
//            if (gamepad1.dpad_down){
//                flywheelSpeed = flywheelSpeed - 5;
//                sleep(250);
//            }
//
//            //speed up
////            if (!currentGamepad1_dpad_up && previousGamepad1_dpad_up) {
////                flywheelSpeed += 250;
////
////                previousGamepad1_dpad_up = currentGamepad1_dpad_up;
////            }
////
////            //speed down
////            if (!currentGamepad1_dpad_down && previousGamepad1_dpad_down) {
////                flywheelSpeed -= 250;
////
////                previousGamepad1_dpad_down = currentGamepad1_dpad_down;
////            }
//
//
//
//            //INTAKE
//            if (gamepad1.left_bumper && !wasPressedIntake) {
//                toggleStateIntake = !toggleStateIntake; // Flip the toggle state
//            }
//            wasPressedIntake = gamepad1.left_bumper;
//            if (toggleStateIntake) {
//                robot.intake.setPower(-1);
//            } else {
//                robot.intake.setPower(0);
//            }
//
//
//            //FLYHWEEL
//
//            if (gamepad1.right_bumper && !wasPressedFlywheel) {
//                toggleStateFlywheel = !toggleStateFlywheel; // Flip the toggle statejkkj
//            }
//            wasPressedFlywheel = gamepad1.right_bumper;
//            double targetTicksPerSec = flywheelSpeed / 60.0 * ticksPerRev;
//
////            double measuredTicksPerSec = robot.flywheelOne.getVelocity();
////            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;
//
//
//
//            if (toggleStateFlywheel) {
//                robot.flywheelOne.setVelocityPIDFCoefficients(P, I, D, F);
//                robot.flywheelTwo.setVelocityPIDFCoefficients(P, I, D, F);
//
//                robot.flywheelOne.setVelocity(targetTicksPerSec);
//                robot.flywheelTwo.setVelocity(targetTicksPerSec);
//
////                robot.flywheelOne.setPower(flywheelSpeed);
////                robot.flywheelTwo.setPower(flywheelSpeed);
//
//            } else {
//                robot.flywheelOne.setVelocity(0);
//                robot.flywheelTwo.setVelocity(0);
//            }
//
//
//
////            if (gamepad1.left_trigger > 0.2) {//intake forward
////                robot.intake.setPower(-1);
////            }
//
//            if (gamepad1.dpad_right) robot.intake.setPower(1);
//
//            if (gamepad1.x) {
//                robot.feedServo.setPosition(0);
//            } else {
//                robot.feedServo.setPosition(1);
//            }
//
////            if (robot.feedServo.getPosition() == 1 && gamepad1.x == true){
////                robot.feedServo.setPosition(0);
////                sleep(500);
////            } else if (robot.feedServo.getPosition() == 0 && gamepad1.x == true){
////                robot.feedServo.setPosition(1);
////                sleep(500);
////            }
//            telemetry.update();
//        }
//    }
//}
