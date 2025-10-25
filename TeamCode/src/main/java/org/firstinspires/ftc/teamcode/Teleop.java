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

        while (opModeIsActive()) {

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
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
            if (gamepad1.y) {//intakec
                robot.flywheelOne.setPower(1);
                robot.flywheelTwo.setPower(1);
            } else if (gamepad1.a) {
                robot.flywheelOne.setPower(0);
                robot.flywheelTwo.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.feedServo.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                robot.feedServo.setPosition(1);
            }
        }
    }
}
