package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMap {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotorEx flywheelOne, flywheelTwo;
    public DcMotor intake;
    public Servo feedServo;
    private DcMotor[] drivemotors;

    public void init(HardwareMap hwMap) {
        frontLeftDrive  = hwMap.get(DcMotor.class, "FLD");
        frontRightDrive = hwMap.get(DcMotor.class, "FRD");
        backLeftDrive   = hwMap.get(DcMotor.class, "BLD");
        backRightDrive  = hwMap.get(DcMotor.class, "BRD");
        feedServo       = hwMap.get(Servo.class, "feedServo");
        flywheelOne     = hwMap.get(DcMotorEx.class, "flywheelOne");
        flywheelTwo     = hwMap.get(DcMotorEx.class, "flywheelTwo");
        intake          = hwMap.get(DcMotor.class, "intake");

        //fl = 0
        //fr = 1
        //bl =  2
        //br = 3


        drivemotors = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, flywheelOne, flywheelTwo};

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheelOne.setDirection(DcMotor.Direction.REVERSE);
        flywheelTwo.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //zero power behavior
        for (DcMotor motor : drivemotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        stopAll();
    }

    public void stopAll() {
        for (DcMotor motor : drivemotors) {
            motor.setPower(0);
            feedServo.setPosition(1);
        }
    }
}