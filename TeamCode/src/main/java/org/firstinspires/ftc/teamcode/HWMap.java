package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMap {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, flywheelOne, flywheelTwo;
    private DcMotor[] motors;

    public void init(HardwareMap hwMap) {
        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hwMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hwMap.get(DcMotor.class, "backRightDrive");
        flywheelOne     = hwMap.get(DcMotor.class, "flywheelOne");
        flywheelTwo     = hwMap.get(DcMotor.class, "flywheelTwo");

        motors = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, flywheelOne, flywheelTwo};

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheelOne.setDirection(DcMotor.Direction.FORWARD);
        flywheelTwo.setDirection(DcMotor.Direction.REVERSE);

        //zero power behavior
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        stopAll();
    }

    public void stopAll() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }
}