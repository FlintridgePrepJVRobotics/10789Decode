package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HWMap;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp
public class apriltagAuto extends LinearOpMode {
    public HWMap robot = new HWMap();

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // PID Controllers for auto-alignment
    // It is recommended to tune these values for your specific robot
    private PIDController strafePID = new PIDController(0.05, 0, 0.002);
    private PIDController forwardPID = new PIDController(0.05, 0, 0.002);
    private PIDController turnPID = new PIDController(0.01, 0, 0.001);

    // Flywheel tuning (assuming these are for a shooter)
    private double flywheelBasePower = 0.002; // Scaling factor for distance → power
    private double flywheelTuning = 1.0;      // Modifier for quick tuning


    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---
        robot.init(hardwareMap);
        // outtake.init(hardwareMap);

        // Vision setup
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true) // Enable live view for easier debugging
                .build();

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        waitForStart();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            /// /write code here do some movement shit ^^^
            aim();
            spoolFlywheel();
//            fire();
            setFlywheel(0);

        }
        // Cleanup resources after the loop finishes
        visionPortal.close();
    }

    public void aim(){
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        AprilTagDetection targetTag = detections.get(0);

        // Define a tolerance. If the error is within this range, consider it "aligned".
        double tolerance = 2.0; // cm for distance, degrees for heading

        double strafeError = targetTag.ftcPose.x;
        double forwardError = 0; //targetTag.ftcPose.y - 15; // Target 15cm away from the tag
        double headingError = targetTag.ftcPose.yaw;

        // Stop the robot if it's close enough to the target
        if (Math.abs(strafeError) < tolerance && Math.abs(forwardError) < tolerance && Math.abs(headingError) < tolerance) {
            stopDriving();
        } else {
            // Otherwise, continue driving towards the target
            drive(strafeError, forwardError, headingError);
        }
    }

//    public void fire(){
//        robot.feedServo.setPosition(1);
//        sleep(500);
//        robot.feedServo.setPosition(0);
//    }

    /**
     * Drives the robot using PID control to correct for errors.
     */
    public void drive(double strafeError, double forwardError, double headingError) {
        double strafePower = strafePID.update(strafeError);
        double forwardPower = forwardPID.update(forwardError);
        double turnPower = turnPID.update(headingError);

        // Mecanum drive calculations
        double fl = forwardPower + strafePower + turnPower;
        double fr = forwardPower - strafePower - turnPower;
        double bl = forwardPower - strafePower + turnPower;
        double br = forwardPower + strafePower - turnPower;

        // Normalize motor powers to ensure they are within [-1, 1]
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        setMotorPowers(fl, fr, bl, br);
    }

    /**
     * Drives the robot based on gamepad inputs.
     */
    public void manualDrive(double forward, double strafe, double turn) {
        // Mecanum drive calculations
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalize motor powers
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        setMotorPowers(fl, fr, bl, br);
    }

    /**
     * A helper method to set all drive motor powers at once.
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        robot.frontLeftDrive.setPower(fl);
        robot.frontRightDrive.setPower(fr);
        robot.backLeftDrive.setPower(bl);
        robot.backRightDrive.setPower(br);
    }

    /**
     * Stops all drive motors.
     */
    public void stopDriving() {
        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Calculates the required flywheel power based on distance and sets it.
     */
    public void spoolFlywheel() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        AprilTagDetection targetTag = detections.get(0);

        double power = targetTag.ftcPose.y * flywheelBasePower * flywheelTuning;
        power = Math.max(0, Math.min(1, power)); // Clamp power between 0 and 1

        robot.flywheelOne.setPower(power);
        robot.flywheelTwo.setPower(power);

        telemetry.addData("Flywheel Power", power);
        telemetry.addData("Target Distance", targetTag.ftcPose.y);
    }

    public void setFlywheel(double fw){
        robot.flywheelOne.setPower(fw);
        robot.flywheelTwo.setPower(fw);
    }
}
