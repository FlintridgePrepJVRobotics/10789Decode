package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMap {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, flywheel, outtake;
    private DcMotor[] motors;

    public void init(HardwareMap hwMap) {
        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hwMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hwMap.get(DcMotor.class, "backRightDrive");
        flywheel        = hwMap.get(DcMotor.class, "flywheel");
        outtake         = hwMap.get(DcMotor.class, "outtake");

        motors = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, flywheel, outtake};

        // Set directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.REVERSE);

        // Apply zero power behavior to all
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