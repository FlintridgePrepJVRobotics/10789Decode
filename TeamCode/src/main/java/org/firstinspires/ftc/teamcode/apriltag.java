package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.PIDController;
import java.util.List;



@TeleOp
public class apriltag extends LinearOpMode {

    // Motors
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Vision
    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;

    PIDController strafePID = new PIDController(0.05, 0, 0.002);
    PIDController forwardPID = new PIDController(0.05, 0, 0.002);
    PIDController turnPID = new PIDController(0.01, 0, 0.001);

    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (gamepad1.a && !detections.isEmpty()) {
                AprilTagDetection target = detections.get(0);

                double strafeError = target.ftcPose.x;   // cm left/right
                double forwardError = target.ftcPose.y;  // cm forward/back
                double headingError = target.ftcPose.yaw; // degrees

                drive(strafeError, forwardError, headingError);
            } else {
                // Normal driver control
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;
                manualDrive(forward, strafe, turn);
            }


            while (!isStopRequested() && opModeIsActive()) {
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);

                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                }
                telemetry.update();
            }
        }
    }

    public void drive(double strafeError, double forwardError, double headingError) {
        double strafePower = strafePID.update(strafeError);
        double forwardPower = forwardPID.update(forwardError);
        double turnPower = turnPID.update(headingError);

        double fl = forwardPower + strafePower + turnPower;
        double fr = forwardPower - strafePower - turnPower;
        double bl = forwardPower - strafePower + turnPower;
        double br = forwardPower + strafePower - turnPower;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    public void manualDrive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }
}