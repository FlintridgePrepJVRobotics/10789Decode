package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWMap;

//name that appeafrss on the driver hub screen
@Autonomous(name = "shootinplace")
public class auton extends LinearOpMode {
    //making a robot from project file (hardware map)
    public HWMap robot = new HWMap();
    double power = 950;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);// start block
        waitForStart();
        robot.flywheelOne.setVelocity(power);
        robot.flywheelTwo.setVelocity(power);
        sleep(3000);
        robot.feedServo.setPosition(0);//number 1
        sleep(2000);
        robot.feedServo.setPosition(1);
        robot.flywheelOne.setPower(0);
        robot.flywheelTwo.setPower(0);
        sleep(1000);
        robot.intake.setPower(0.5);
        sleep(250);
        robot.intake.setPower(-0.5);
        sleep(500);
        robot.intake.setPower(0);//end block


        robot.flywheelOne.setVelocity(power);
        robot.flywheelTwo.setVelocity(power);
        sleep(3000);
        robot.feedServo.setPosition(0);//number 1
        sleep(2000);
        robot.feedServo.setPosition(1);
        robot.flywheelOne.setPower(0);
        robot.flywheelTwo.setPower(0);
        sleep(1000);
        robot.intake.setPower(0.5);
        sleep(250);
        robot.intake.setPower(-1);
        sleep(500);
        robot.intake.setPower(0);//end block

        robot.flywheelOne.setVelocity(power);
        robot.flywheelTwo.setVelocity(power);
        sleep(3000);
        robot.feedServo.setPosition(0);//number 1
        sleep(2000);
        robot.feedServo.setPosition(1);
        robot.flywheelOne.setPower(0);
        robot.flywheelTwo.setPower(0);
        sleep(1000);
        robot.intake.setPower(0.5);
        sleep(250);
        robot.intake.setPower(-1);
        sleep(500);
        robot.intake.setPower(0);//end blo ck






//        forward(250,1);iyt
//        sleep(500);
//        forward(250,-1);
//        robot.feedServo.setPosition(0);//number 2
//        sleep(3000);
//        robot.feedServo.setPosition(1);
//        sleep(3000);
//        robot.intake.setPower(0.5);
//        sleep(250);
//        robot.intake.setPower(-1);
//        sleep(2000);
//        forward(250,1); out
//        sleep(500);
//        forward(250,-1);
//        robot.intake.setPower(-0);
//        sleep(1000);
//        robot.feedServo.setPosition(0);//number 3
//        sleep(2000);
//        robot.flywheelOne.setPower(0);
//        robot.flywheelTwo.setPower(0);
//        robot.feedServo.setPosition(0);
        robot.backRightDrive.setPower(-0.7);
        robot.backLeftDrive.setPower(-0.7);
        robot.frontRightDrive.setPower(-0.7);
        robot.frontLeftDrive.setPower(-0.7);
        sleep(4000);
        stopall();
    }

    public void forward(int time, double speed) { //forward
        robot.backRightDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
        sleep(time);
        stopall();
    }
    public void stopall() { //forward
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
    }

}
