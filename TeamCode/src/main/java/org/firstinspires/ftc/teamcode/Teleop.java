package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "2025Decode10789")
public class Teleop extends LinearOpMode {
    public HWMap robot = new HWMap();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        double speed = 1;

        boolean previousGamepad1_dpad_up = false;
        boolean previousGamepad1_dpad_down = false;

        int ticksPerRev =28;
        double flywheelSpeed = 20;

        double P = 10.0;
        double I = 3.0;
        double D = 0.0;
        double F = 12.0;


        boolean toggleStateFlywheel = false; // The variab le that holds the toggled state (e.g., claw open/closed)
        boolean wasPressedFlywheel = false;  // The variable to store the button's state from the previous cycle
        boolean toggleStateIntake = false; // The variable that holds the toggled state (e.g., claw open/closed)
        boolean wasPressedIntake = false;  // The variable to store the button's state from the previous cycle

        int checkInterval = 200;
        int prevPositionnew1 = robot.flywheelOne.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        int prevPositionnew2 = robot.flywheelTwo.getCurrentPosition();
        ElapsedTime timer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer1.reset();

        while (opModeIsActive()) {

            boolean currentGamepad1_dpad_up = gamepad1.dpad_up;
            boolean currentGamepad1_dpad_down = gamepad1.dpad_down;

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = -gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeftDrive.setPower(frontLeftPower * speed);
            robot.backLeftDrive.setPower(backLeftPower * speed);
            robot.frontRightDrive.setPower(frontRightPower * speed);
            robot.backRightDrive.setPower(backRightPower * speed);


            telemetry.addData("flywheel speed", flywheelSpeed);


//            if (timer.time() > checkInterval) {
//                    double speednew1 = (double) (robot.flywheelOne.getCurrentPosition() - prevPositionnew1) / timer.time();
//
//                    // This will print out the ticks per millisecond of the motor.
//                    telemetry.addData("Ticks per ms for motor 1", speednew1);
//                    telemetry.update();
//                    prevPositionnew1 = robot.flywheelOne.getCurrentPosition();
//                    timer.reset();
//
//            }

//            if (timer1.time() > checkInterval) {
//                    double speednew2 = (double) (robot.flywheelTwo.getCurrentPosition() - prevPositionnew2) / timer1.time();
//
//                    // This will print out the ticks per millisecond of the motor.
//                    telemetry.addData("Ticks per ms for motor 2", speednew2);
//                    telemetry.update();
//                    prevPositionnew2 = robot.flywheelTwo.getCurrentPosition();
//                    timer.reset();
//
//            }

            if (gamepad1.dpad_up){
                flywheelSpeed = flywheelSpeed + 5;
                sleep(250);

            }
            if (gamepad1.dpad_down){
                flywheelSpeed = flywheelSpeed - 5;
                sleep(250);
            }

            //speed up
//            if (!currentGamepad1_dpad_up && previousGamepad1_dpad_up) {
//                flywheelSpeed += 250;
//
//                previousGamepad1_dpad_up = currentGamepad1_dpad_up;
//            }
//
//            //speed down
//            if (!currentGamepad1_dpad_down && previousGamepad1_dpad_down) {
//                flywheelSpeed -= 250;
//
//                previousGamepad1_dpad_down = currentGamepad1_dpad_down;
//            }



            //INTAKE
            if (gamepad1.left_bumper && !wasPressedIntake) {
                toggleStateIntake = !toggleStateIntake; // Flip the toggle state
            }
            wasPressedIntake = gamepad1.left_bumper;
            if (toggleStateIntake) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }


            //FLYHWEEL

            if (gamepad1.right_bumper && !wasPressedFlywheel) {
                toggleStateFlywheel = !toggleStateFlywheel; // Flip the toggle statejkkj
            }
            wasPressedFlywheel = gamepad1.right_bumper;
            double targetTicksPerSec = flywheelSpeed / 60.0 * ticksPerRev;

//            double measuredTicksPerSec = robot.flywheelOne.getVelocity();
//            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;



            if (toggleStateFlywheel) {
                robot.flywheelOne.setVelocityPIDFCoefficients(P, I, D, F);
                robot.flywheelTwo.setVelocityPIDFCoefficients(P, I, D, F);

                robot.flywheelOne.setVelocity(28);
                robot.flywheelTwo.setVelocity(28);

//                robot.flywheelOne.setPower(flywheelSpeed);
//                robot.flywheelTwo.setPower(flywheelSpeed);

            } else {
                robot.flywheelOne.setVelocity(0);
                robot.flywheelTwo.setVelocity(0);
            }



//            if (gamepad1.left_trigger > 0.2) {//intake forward
//                robot.intake.setPower(-1);
//            }

            if (gamepad1.dpad_right) robot.intake.setPower(1);

            if (gamepad1.x) {
                robot.feedServo.setPosition(0);
            } else {
                robot.feedServo.setPosition(1);
            }

//            if (robot.feedServo.getPosition() == 1 && gamepad1.x == true){
//                robot.feedServo.setPosition(0);
//                sleep(500);
//            } else if (robot.feedServo.getPosition() == 0 && gamepad1.x == true){
//                robot.feedServo.setPosition(1);
//                sleep(500);
//            }
            telemetry.update();
        }
    }
}
//6767676767676767767667676776