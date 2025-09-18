package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.HWMap;
import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMap;

@TeleOp(name = "2025Decode10789")
public class Teleop extends LinearOpMode {
    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        boolean isSpinning = false;
        double speed = 1;
        boolean open = true;
        boolean slowMode = false;
        boolean slowModeToggle = false;

        while (opModeIsActive()) {


            boolean aButtonHeld = false;
            double rightPower = gamepad1.right_stick_y;
            double leftPower = gamepad1.left_stick_y;

            robot.rightDrive.setPower(rightPower * speed);
            robot.leftDrive.setPower(leftPower * speed);
        }
    }
}
