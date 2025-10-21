package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMap {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotor flywheelOne, flywheelTwo;
    //public servo feedServo;
    private DcMotor[] drivemotors;

    public void init(HardwareMap hwMap) {
        frontLeftDrive  = hwMap.get(DcMotor.class, "FLD");
        frontRightDrive = hwMap.get(DcMotor.class, "FRD");
        backLeftDrive   = hwMap.get(DcMotor.class, "BLD");
        backRightDrive  = hwMap.get(DcMotor.class, "BRD");
//        feedServo       = hwMap.get(Servo.class, "feedServo")
        flywheelOne     = hwMap.get(DcMotor.class, "flywheelOne");
        flywheelTwo     = hwMap.get(DcMotor.class, "flywheelTwo");

        //fl = 0
        //fr = 1
        //bl = 2
        //br = 3


        drivemotors = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //feedServo.setPosition(0);
        flywheelOne.setDirection(DcMotor.Direction.FORWARD);
        flywheelTwo.setDirection(DcMotor.Direction.REVERSE);

        //zero power behavior
        for (DcMotor motor : drivemotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        stopAll();
    }

    public void stopAll() {
        for (DcMotor motor : drivemotors) {
            motor.setPower(0);
        }
    }
}