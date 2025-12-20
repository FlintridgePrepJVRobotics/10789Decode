package org.firstinspires.ftc.teamcode.Autonomous;
import org.firstinspires.ftc.teamcode.HWMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "CLOSAERAUTON")
public class closeauton extends LinearOpMode {

    public HWMap robot = new HWMap();

    // ===== SHOOTER CONSTANTS (same as TeleOp) =====
    private static final double MOTOR_TICKS_PER_REV = 560.0;
    private static final double MOTOR_MAX_RPM = 300.0;
    private static final double SHOOTER_TO_MOTOR_RATIO = 4.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        DcMotorEx flywheelOne = (DcMotorEx) robot.flywheelOne;
        DcMotorEx flywheelTwo = (DcMotorEx) robot.flywheelTwo;

        // Shooter encoder setup
        flywheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Same PIDF as TeleOp
        flywheelOne.setVelocityPIDFCoefficients(10, 0.5, 0, 12);
        flywheelTwo.setVelocityPIDFCoefficients(10, 0.5, 0, 12);

        waitForStart();
        if (!opModeIsActive()) return;

        // ================= SPIN UP SHOOTER =================
        double shooterRPM = 360;

        double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
        motorRPM = Math.min(motorRPM, MOTOR_MAX_RPM);

        double targetTicksPerSec =
                motorRPM * MOTOR_TICKS_PER_REV / 60.0;

        flywheelOne.setVelocity(targetTicksPerSec);
        flywheelTwo.setVelocity(targetTicksPerSec);

        sleep(1200); // initial spin-up

        // ================= SHOOT 3 RINGS =================
        for (int i = 0; i < 3; i++) {

            sleep(2500);

            // Feed ring into shooter
            robot.feedServo.setPosition(0);
            sleep(1600);// slow servo push
            robot.feedServo.setPosition(1);
            sleep(1600);


            // Run intake to load ring
            robot.intake.setPower(-0.65);
            sleep(500);          // intake time to move ring up
            robot.intake.setPower(0);

        }

        // ================= SHUT DOWN =================
        robot.intake.setPower(0);
        flywheelOne.setVelocity(0);
        flywheelTwo.setVelocity(0);
    }
}