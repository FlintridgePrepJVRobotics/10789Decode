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
public class apriltagTeleop extends LinearOpMode {
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

    // State variable to track if flywheel is ready
    private boolean spooled = false;

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
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            // Check if 'A' button is pressed AND at least one tag is detected
            if (gamepad1.a && !detections.isEmpty()) {
                // --- AUTO-ALIGNMENT MODE ---
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

            } else {
                // --- MANUAL CONTROL MODE ---
                double forward = -gamepad1.right_stick_y;
                double strafe = gamepad1.right_stick_x;
                double turn = gamepad1.left_stick_x;

                manualDrive(forward, strafe, turn);

                // Stop the flywheel when not auto-aligning
//                stopFlywheel(); // Uncomment if you have flywheel motors mapped
//                spooled = false;
            }

            if (gamepad1.right_bumper && !detections.isEmpty()){
                AprilTagDetection targetTag = detections.get(0);
                spool(targetTag.ftcPose.z); // Spool up the flywheel based on distance
            } else{
                this.spooled = false;
            }

            if(gamepad1.left_bumper && spooled){
                robot.feedServo.setPosition(1);
            } else{
                robot.feedServo.setPosition(0);
            }

            // --- TELEMETRY ---
            telemetry.addData("Mode", gamepad1.a ? "AUTO-ALIGN" : "MANUAL");
            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X Error (strafe)", tag.ftcPose.x);
                telemetry.addData("Y Error (forward)", tag.ftcPose.y);
                telemetry.addData("Yaw Error (turn)", tag.ftcPose.yaw);
                telemetry.addData("Is Spooled", spooled);
            } else {
                telemetry.addLine("No AprilTag detected.");
            }
            telemetry.update();
        }

        // Cleanup resources after the loop finishes
        visionPortal.close();
    }

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
    public void spool(double distanceCm) {
        double power = distanceCm * flywheelBasePower * flywheelTuning;
        power = Math.max(0, Math.min(1, power)); // Clamp power between 0 and 1

        robot.flywheelOne.setPower(power);
        robot.flywheelTwo.setPower(power);

        this.spooled = true; // Set the class-level 'spooled' variable

        telemetry.addData("Flywheel Power", power);
        telemetry.addData("Target Distance", distanceCm);
    }
}
