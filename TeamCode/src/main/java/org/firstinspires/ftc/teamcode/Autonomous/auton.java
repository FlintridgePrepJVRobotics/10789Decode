package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWMap;

//name that appears on the driver hub screen
@Autonomous(name = "shootinplace")
public class auton extends LinearOpMode {
    //making a robot from project file (hardware map)
    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        waitForStart();
        robot.flywheelOne.setPower(0.5);
        robot.flywheelTwo.setPower(0.5);
        sleep(3000);
        robot.feedServo.setPosition(1);//number 1
        sleep(2000);
        robot.feedServo.setPosition(0);
        sleep(2000);
        forward(250,1);
        forward(100,-1);
        robot.intake.setPower(-0.5);
        sleep(1000);
        robot.feedServo.setPosition(1);//number 2
        sleep(2000);
        robot.feedServo.setPosition(0);
        sleep(2000);
        forward(250,1);
        forward(100,-1);
        robot.intake.setPower(-0.5);
        sleep(1000);
        robot.feedServo.setPosition(1);//number 3
        sleep(2000);
        robot.feedServo.setPosition(0);
        sleep(2000);
        robot.flywheelOne.setPower(0);
        robot.flywheelTwo.setPower(0);
        robot.feedServo.setPosition(0);
        forward(6700, 0.4);
        stopall();
    }

    public void forward(int time, double speed) { //forward
        robot.backRightDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
        sleep(time);
    }
    public void stopall() { //forward
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
    }

}
