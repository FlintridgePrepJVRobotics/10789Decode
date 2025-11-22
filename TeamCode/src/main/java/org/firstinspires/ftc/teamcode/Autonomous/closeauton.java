package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWMap;

//name that appearss on the driver hub screen
@Autonomous(name = "closeatuon")
public class closeauton extends LinearOpMode {
    //making a robot from project file (hardware map)
    public HWMap robot = new HWMap();
    double power = 0.45;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);// start block
        waitForStart();

        forward(5700,-0.2);

        robot.flywheelOne.setPower(power);
        robot.flywheelTwo.setPower(power);
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


        robot.flywheelOne.setPower(power);
        robot.flywheelTwo.setPower(power);
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

        robot.flywheelOne.setPower(power);
        robot.flywheelTwo.setPower(power);
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
