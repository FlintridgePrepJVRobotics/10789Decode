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

@Autonomous(name = "SHootinplace")
public class auton extends LinearOpMode {

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
        double shooterRPM = 400;

        double motorRPM = shooterRPM / SHOOTER_TO_MOTOR_RATIO;
        motorRPM = Math.min(motorRPM, MOTOR_MAX_RPM);

        double targetTicksPerSec =
                motorRPM * MOTOR_TICKS_PER_REV / 60.0;

        flywheelOne.setVelocity(targetTicksPerSec);
        flywheelTwo.setVelocity(targetTicksPerSec);

        sleep(1200); // initial spin-up

        // ================= SHOOT 3 RINGS =================
        for (int i = 0; i < 4; i++) {

            sleep(2500);

            // Feed ring into shooter
            robot.feedServo.setPosition(0);
            sleep(2000);// slow servo push

            // Run intake to load ring
            robot.intake.setPower(-0.85);


            robot.feedServo.setPosition(1);
            sleep(1600);



            sleep(500);          // intake time to move ring up
            robot.intake.setPower(0);

        }

        // ================= SHUT DOWN =================
        robot.intake.setPower(0);
        flywheelOne.setVelocity(0);
        flywheelTwo.setVelocity(0);

    }
}
//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.HWMap;
//
////name that appeafrss on the driver hub screen
//@Autonomous(name = "shootinplace")
//public class auton extends LinearOpMode {
//    //making a robot from project file (hardware map)
//    public HWMap robot = new HWMap();
//    double power = 950;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //initialize hardware map
//        robot.init(hardwareMap);// start block
//        waitForStart();
//        robot.flywheelOne.setVelocity(power);
//        robot.flywheelTwo.setVelocity(power);
//        sleep(3000);
//        robot.feedServo.setPosition(0);//number 1
//        sleep(2000);
//        robot.feedServo.setPosition(1);
//        robot.flywheelOne.setPower(0);
//        robot.flywheelTwo.setPower(0);
//        sleep(1000);
//        robot.intake.setPower(0.5);
//        sleep(250);
//        robot.intake.setPower(-0.5);
//        sleep(500);
//        robot.intake.setPower(0);//end block
//
//
//        robot.flywheelOne.setVelocity(power);
//        robot.flywheelTwo.setVelocity(power);
//        sleep(3000);
//        robot.feedServo.setPosition(0);//number 1
//        sleep(2000);
//        robot.feedServo.setPosition(1);
//        robot.flywheelOne.setPower(0);
//        robot.flywheelTwo.setPower(0);
//        sleep(1000);
//        robot.intake.setPower(0.5);
//        sleep(250);
//        robot.intake.setPower(-1);
//        sleep(500);
//        robot.intake.setPower(0);//end block
//
//        robot.flywheelOne.setVelocity(power);
//        robot.flywheelTwo.setVelocity(power);
//        sleep(3000);
//        robot.feedServo.setPosition(0);//number 1
//        sleep(2000);
//        robot.feedServo.setPosition(1);
//        robot.flywheelOne.setPower(0);
//        robot.flywheelTwo.setPower(0);
//        sleep(1000);
//        robot.intake.setPower(0.5);
//        sleep(250);
//        robot.intake.setPower(-1);
//        sleep(500);
//        robot.intake.setPower(0);//end blo ck
//
//
//
//
//
//
////        forward(250,1);iyt
////        sleep(500);
////        forward(250,-1);
////        robot.feedServo.setPosition(0);//number 2
////        sleep(3000);
////        robot.feedServo.setPosition(1);
////        sleep(3000);
////        robot.intake.setPower(0.5);
////        sleep(250);
////        robot.intake.setPower(-1);
////        sleep(2000);
////        forward(250,1); out
////        sleep(500);
////        forward(250,-1);
////        robot.intake.setPower(-0);
////        sleep(1000);
////        robot.feedServo.setPosition(0);//number 3
////        sleep(2000);
////        robot.flywheelOne.setPower(0);
////        robot.flywheelTwo.setPower(0);
////        robot.feedServo.setPosition(0);
//        robot.backRightDrive.setPower(-0.7);
//        robot.backLeftDrive.setPower(-0.7);
//        robot.frontRightDrive.setPower(-0.7);
//        robot.frontLeftDrive.setPower(-0.7);
//        sleep(4000);
//        stopall();
//    }
//
//    public void forward(int time, double speed) { //forward
//        robot.backRightDrive.setPower(speed);
//        robot.backLeftDrive.setPower(speed);
//        robot.frontRightDrive.setPower(speed);
//        robot.frontLeftDrive.setPower(speed);
//        sleep(time);
//        stopall();
//    }
//    public void stopall() { //forward
//        robot.backRightDrive.setPower(0);
//        robot.backLeftDrive.setPower(0);
//        robot.frontRightDrive.setPower(0);
//        robot.frontLeftDrive.setPower(0);
//    }
//
//}
