package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMap;

//name that appears on the driver hub screen
@Autonomous(name = "Basic Auto")
public class auton extends LinearOpMode {
    //making a robot from project file (hardware map)
    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware maphnjghjkghjhjk
        robot.init(hardwareMap);

        waitForStart();
        forward(2500, 1);
        forward(500, 0);
    }

    public void forward(int time, int speed) { //forward
        robot.backRightDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
        sleep(time);
    }


    }
