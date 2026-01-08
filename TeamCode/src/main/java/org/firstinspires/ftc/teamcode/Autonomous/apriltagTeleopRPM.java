package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HWMap;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "apriltagTeleopRPM")
public class apriltagTeleopRPM extends LinearOpMode {
    public HWMap robot = new HWMap();

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // PID Controllers for auto-alignment
    private PIDController strafePID = new PIDController(0.05, 0, 0.002);
    private PIDController forwardPID = new PIDController(0.05, 0, 0.002);
    private PIDController turnPID = new PIDController(0.01, 0, 0.001);

    // Linear power constants (unused for RPM spool but kept for reference)
    private static final double POWER_SLOPE = 0.00075;
    private static final double POWER_INTERCEPT = 0.2775;

    // RPM / motor constants (match your Teleop)
    private static final double MOTOR_TICKS_PER_REV = 560.0;
    private static final double MOTOR_MAX_RPM      = 300.0; // motor RPM max
    private static final double SHOOTER_TO_MOTOR_RATIO = 4.0; // flywheelRPM = motorRPM * ratio

    // PIDF coefficients for motor velocity control (starting points)
    private static final double PIDF_P = 10.0;
    private static final double PIDF_I = 0.5;
    private static final double PIDF_D = 0.0;
    private static final double PIDF_F = 12.0;

    // Shooter presets (flywheel RPM)
    private static final double RPM_CLOSE = 360.0;
    private static final double RPM_FAR   = 400.0;

    // Distance threshold (cm) to decide close vs far
    private static final double DISTANCE_THRESHOLD_CM = 100.0; // 1 meter

    // State
    private boolean shooterOn = false;       // whether flywheel is on
    private boolean wasPressedTrigger = false; // for toggling shooterOn via right trigger
    private boolean rpmIsFar = true;         // which preset is selected manually (true -> FAR 400, false -> CLOSE 360)
    private boolean wasPressedRPMtoggle = false; // for right bumper RPM toggle

    // For DcMotorEx
    private DcMotorEx flywheelOneEx = null;
    private DcMotorEx flywheelTwoEx = null;

    // Tracks whether auto-mode is active (a button)
    private boolean autoAlignMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Setup vision
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        // Cast flywheels to DcMotorEx and set encoder mode + PIDF
        flywheelOneEx = (DcMotorEx) robot.flywheelOne;
        flywheelTwoEx = (DcMotorEx) robot.flywheelTwo;

        flywheelOneEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwoEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelOneEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTwoEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelOneEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTwoEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply PIDF coefficients (tune these later)
        flywheelOneEx.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
        flywheelTwoEx.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.addLine("Controls:");
        telemetry.addLine("- Hold A: Auto-align");
        telemetry.addLine("- Right trigger: Toggle flywheel ON/OFF");
        telemetry.addLine("- Right bumper (manual mode): toggle 360 <-> 400 RPM");
        telemetry.addLine("- Left bumper: feed servo (only when shooter ON)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            // Read buttons
            autoAlignMode = gamepad1.a;

            // Toggle shooter ON/OFF with right trigger button press (press once to toggle)
            boolean currentTriggerPressed = gamepad1.right_trigger > 0.5;
            if (currentTriggerPressed && !wasPressedTrigger) {
                shooterOn = !shooterOn;
            }
            wasPressedTrigger = currentTriggerPressed;

            // Manual RPM toggle: right bumper toggles selected preset when NOT auto-aligning
            if (!autoAlignMode) {
                if (gamepad1.right_bumper && !wasPressedRPMtoggle) {
                    rpmIsFar = !rpmIsFar; // toggle between FAR(400) and CLOSE(360)
                }
                wasPressedRPMtoggle = gamepad1.right_bumper;
            } else {
                // reset rpm toggle press tracking while in auto mode
                wasPressedRPMtoggle = false;
            }

            // Auto-align controls (hold A to auto-align)
            if (autoAlignMode && !detections.isEmpty()) {
                AprilTagDetection targetTag = detections.get(0);

                double tolerance = 2.0; // cm/degrees tolerance

                double strafeError = targetTag.ftcPose.x;          // x (cm) lateral error
                double forwardError = 0;                           // keep forward locked in this routine (optional)
                double headingError = targetTag.ftcPose.yaw;      // yaw (deg)

                if (Math.abs(strafeError) < tolerance && Math.abs(headingError) < tolerance) {
                    stopDriving();
                } else {
                    drive(strafeError, forwardError, headingError);
                }

                // If shooter is ON, select RPM based on tag distance (z)
                if (shooterOn) {
                    double distCm = targetTag.ftcPose.z;
                    double chosenRPM = (distCm < DISTANCE_THRESHOLD_CM) ? RPM_CLOSE : RPM_FAR;
                    setFlywheelRPM(chosenRPM);
                    // keep rpmIsFar in sync with auto selection for telemetry clarity
                    rpmIsFar = (chosenRPM == RPM_FAR);
                } else {
                    // shooter OFF - ensure motors zero velocity
                    setFlywheelVelocityTicksPerSec(0);
                }

            } else {
                // Manual driving
                double forward = -gamepad1.right_stick_y;
                double strafe = gamepad1.right_stick_x;
                double turn = gamepad1.left_stick_x;
                manualDrive(forward, strafe, turn);

                // Manual shooter control:
                if (shooterOn) {
                    double chosenRPM = rpmIsFar ? RPM_FAR : RPM_CLOSE;
                    setFlywheelRPM(chosenRPM);
                } else {
                    setFlywheelVelocityTicksPerSec(0);
                }
            }

            // Feed servo: left bumper activates feed when shooter is ON
            if (gamepad1.left_bumper && shooterOn) {
                robot.feedServo.setPosition(1.0); // adjust positions to your configuration
            } else {
                robot.feedServo.setPosition(0.0);
            }

            // Telemetry: report mode, rpm selection, shooter state, and tag info
            telemetry.addData("Mode", autoAlignMode ? "AUTO-ALIGN (A held)" : "MANUAL");
            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("Selected Preset", rpmIsFar ? "FAR (400 RPM)" : "CLOSE (360 RPM)");

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X (strafe err, cm)", tag.ftcPose.x);
                telemetry.addData("Y (unused)", tag.ftcPose.y);
                telemetry.addData("Z (distance cm)", tag.ftcPose.z);
                telemetry.addData("Yaw (deg)", tag.ftcPose.yaw);
            } else {
                telemetry.addLine("No AprilTag detected.");
            }

            // Flywheel raw telemetry (if available)
            double fly1_tps = flywheelOneEx.getVelocity();
            double fly2_tps = flywheelTwoEx.getVelocity();

            double fly1_motorRPM = fly1_tps * 60.0 / MOTOR_TICKS_PER_REV;
            double fly2_motorRPM = fly2_tps * 60.0 / MOTOR_TICKS_PER_REV;

            double fly1_shooterRPM = fly1_motorRPM * SHOOTER_TO_MOTOR_RATIO;
            double fly2_shooterRPM = fly2_motorRPM * SHOOTER_TO_MOTOR_RATIO;

            telemetry.addData("Fly1 shooter RPM (est)", String.format("%.1f", fly1_shooterRPM));
            telemetry.addData("Fly2 shooter RPM (est)", String.format("%.1f", fly2_shooterRPM));
            telemetry.update();
        }

        // Cleanup
        visionPortal.close();
        // Ensure flywheels stop
        setFlywheelVelocityTicksPerSec(0);
    }

    // ----------------- Drive helpers -----------------

    public void drive(double strafeError, double forwardError, double headingError) {
        double strafePower = strafePID.update(strafeError);
        double forwardPower = forwardPID.update(forwardError);
        double turnPower = turnPID.update(headingError);

        double fl = forwardPower + strafePower + turnPower;
        double fr = forwardPower - strafePower - turnPower;
        double bl = forwardPower - strafePower + turnPower;
        double br = forwardPower + strafePower - turnPower;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        setMotorPowers(fl, fr, bl, br);
    }

    public void manualDrive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        setMotorPowers(fl, fr, bl, br);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        robot.frontLeftDrive.setPower(fl);
        robot.frontRightDrive.setPower(fr);
        robot.backLeftDrive.setPower(bl);
        robot.backRightDrive.setPower(br);
    }

    public void stopDriving() {
        setMotorPowers(0, 0, 0, 0);
    }

    // ----------------- Flywheel helpers -----------------

    /**
     * Set flywheel using a *flywheel RPM* target (not motor RPM).
     * This converts flywheelRPM -> motorRPM -> ticks/sec and sets DcMotorEx.setVelocity().
     */
    private void setFlywheelRPM(double flywheelRPM) {
        // Convert shooterRPM (flywheel) → motorRPM using gear ratio
        double motorRPM = flywheelRPM / SHOOTER_TO_MOTOR_RATIO;
        if (motorRPM > MOTOR_MAX_RPM) motorRPM = MOTOR_MAX_RPM; // clamp

        // Convert motorRPM → ticks per second
        double targetTicksPerSec = motorRPM * MOTOR_TICKS_PER_REV / 60.0;

        setFlywheelVelocityTicksPerSec(targetTicksPerSec);
    }

    /**
     * Set both flywheels to a target velocity in ticks per second
     */
    private void setFlywheelVelocityTicksPerSec(double ticksPerSec) {
        if (flywheelOneEx != null && flywheelTwoEx != null) {
            flywheelOneEx.setVelocity(ticksPerSec);
            flywheelTwoEx.setVelocity(ticksPerSec);
        }
    }
}
//package org.firstinspires.ftc.teamcode.Debug;
//
//import org.firstinspires.ftc.teamcode.HWMap;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(name = "ENCODER TROUBLESHOOTER", group = "Debug")
//public class apriltagTeleopRPM extends LinearOpMode {
//
//    HWMap robot = new HWMap();
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
//
//        DcMotor fl = robot.frontLeftDrive;
//        DcMotor fr = robot.frontRightDrive;
//        DcMotor bl = robot.backLeftDrive;
//        DcMotor br = robot.backRightDrive;
//
//        // Use raw power, no encoder correction
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        telemetry.addLine("ENCODER TROUBLESHOOTER READY");
//        telemetry.addLine("A=FL | B=FR | X=BL | Y=BR");
//        telemetry.addLine("Positive power SHOULD increase encoder count");
//        telemetry.update();
//
//        waitForStart();
//
//        int lastFL = fl.getCurrentPosition();
//        int lastFR = fr.getCurrentPosition();
//        int lastBL = bl.getCurrentPosition();
//        int lastBR = br.getCurrentPosition();
//
//        while (opModeIsActive()) {
//
//            // Stop all by default
//            fl.setPower(0);
//            fr.setPower(0);
//            bl.setPower(0);
//            br.setPower(0);
//
//            // Drive ONE motor at a time
//            if (gamepad1.a) fl.setPower(0.4);
//            if (gamepad1.b) fr.setPower(0.4);
//            if (gamepad1.x) bl.setPower(0.4);
//            if (gamepad1.y) br.setPower(0.4);
//
//            int curFL = fl.getCurrentPosition();
//            int curFR = fr.getCurrentPosition();
//            int curBL = bl.getCurrentPosition();
//            int curBR = br.getCurrentPosition();
//
//            telemetry.addLine("Press and hold ONE button");
//            telemetry.addLine("----------------------------");
//
//            telemetry.addData("FL encoder", curFL);
//            telemetry.addData("FR encoder", curFR);
//            telemetry.addData("BL encoder", curBL);
//            telemetry.addData("BR encoder", curBR);
//
//            telemetry.addLine("----------------------------");
//            telemetry.addData("ΔFL", curFL - lastFL);
//            telemetry.addData("ΔFR", curFR - lastFR);
//            telemetry.addData("ΔBL", curBL - lastBL);
//            telemetry.addData("ΔBR", curBR - lastBR);
//
//            telemetry.addLine("----------------------------");
//            telemetry.addLine("EXPECTED: encoder INCREASES with +power");
//
//            telemetry.update();
//
//            lastFL = curFL;
//            lastFR = curFR;
//            lastBL = curBL;
//            lastBR = curBR;
//
//            sleep(100);
//        }
//    }
//}
