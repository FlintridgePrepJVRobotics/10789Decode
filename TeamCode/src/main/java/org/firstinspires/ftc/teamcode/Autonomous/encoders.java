package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMap;

@Autonomous(name="encoders")
public class encoders extends LinearOpMode {

    public HWMap robot = new HWMap();

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

        while (robot.backRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.frontLeftDrive.isBusy()){

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

        // Reset encoders
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Optional: show starting encoder values
        telemetry.addData("Start", "FL:%d FR:%d BL:%d BR:%d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.backLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // EXAMPLE MOVE:
        // NOTE: Your method order is (fRight, fLeft, bRight, bLeft).
        // If you want all wheels same direction, keep the signs the same:

        // Small pause so you can read end telemetry


        encoderDrive(0.2, -1000, -1000, -1000, -1000);

        /**
         * encoderDrive(speed, fRightCounts, fLeftCounts, bRightCounts, bLeftCounts)
         * Direction comes from the TARGET POSITION (encoder sign), since power is abs(speed).
         */

        // Use OR so telemetry keeps updating as long as ANY motor is still mo

        // Go back to encoder mode
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Show final encoder values
        telemetry.addData("Done", "FL:%d FR:%d BL:%d BR:%d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.backLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();
    }

}
