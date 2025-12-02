package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "2025Decode10789")
public class Teleop extends LinearOpMode {
    public HWMap robot = new HWMap();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        double speed = 1;
        double flywheelSpeed = 1000;

        boolean toggleStateFlywheel = false; // The variable that holds the toggled state (e.g., claw open/closed)
        boolean wasPressedFlywheel = false;  // The variable to store the button's state from the previous cycle
        boolean toggleStateIntake = false; // The variable that holds the toggled state (e.g., claw open/closed)
        boolean wasPressedIntake = false;  // The variable to store the button's state from the previous cycle

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeftDrive.setPower(frontLeftPower * speed);
            robot.backLeftDrive.setPower(backLeftPower * speed);
            robot.frontRightDrive.setPower(frontRightPower * speed);
            robot.backRightDrive.setPower(backRightPower * speed);
            //to debug: dpad motor control.
//            if (gamepad1.dpad_left){
//                robot.frontLeftDrive.setPower(1);
//            }
//            if (gamepad1.dpad_right){
//                robot.frontRightDrive.setPower(1);
//            }
//            if (gamepad1.dpad_up){
//                robot.backLeftDrive.setPower(1);
//            }
//            if (gamepad1.dpad_down){
//                robot.backRightDrive.setPower(1);
//            }
//            if (gamepad1.y) {//intake
//                robot.flywheelOne.setPower(1);
//                robot.flywheelTwo.setPower(1);
//            } else if (gamepad1.a) {
//                robot.flywheelOne.setPower(0);
//                robot.flywheelTwo.setPower(0);
//            }
            //dfghjkl
            telemetry.addData("flywheel speed", flywheelSpeed);

            if (gamepad1.dpad_up){
                flywheelSpeed = flywheelSpeed + 250;
                sleep(300);

            }
            if (gamepad1.dpad_down){
                flywheelSpeed = flywheelSpeed - 250;
                sleep(300);
            }

            //INTAKE
            // Check if the button is currently pressed AND it was NOT pressed last cycle
            if (gamepad1.left_bumper && !wasPressedIntake) {
                // This condition is true only at the exact moment the button is pressed down
                toggleStateIntake = !toggleStateIntake; // Flip the toggle state
            }

// Update the 'wasPressed' variable for the next loop iteration
            wasPressedIntake = gamepad1.left_bumper;

// Use the toggleState variable to control an action
            if (toggleStateIntake) {
                // Code to run when the toggle is true (e.g., open a claw, turn on an intake)
                robot.intake.setPower(-1);
            } else {
                // Code to run when the toggle is false (e.g., close a claw, turn off an intake)
                robot.intake.setPower(0);
            }


            //FLYHWEEL
            // Check if the button is currently pressed AND it was NOT pressed last cycle
            if (gamepad1.right_bumper && !wasPressedFlywheel) {
                // This condition is true only at the exact moment the button is pressed down
                toggleStateFlywheel = !toggleStateFlywheel; // Flip the toggle state
            }

// Update the 'wasPressed' variable for the next loop iteration
            wasPressedFlywheel = gamepad1.right_bumper;

// Use the toggleState variable to control an action
            if (toggleStateFlywheel) {
                robot.flywheelOne.setVelocity(flywheelSpeed);
                robot.flywheelTwo.setVelocity(flywheelSpeed);
            } else {
                robot.flywheelOne.setPower(0);
                robot.flywheelTwo.setPower(0);
            }



            if (gamepad1.left_trigger > 0.2) {//intake
                robot.intake.setPower(-1);
            }

            if (gamepad1.dpad_right) {//intake
                robot.intake.setPower(1);
            }

            if (gamepad1.x) {
                robot.feedServo.setPosition(0);
            } else {
                robot.feedServo.setPosition(1);
            }
            telemetry.update();
        }
    }
}
