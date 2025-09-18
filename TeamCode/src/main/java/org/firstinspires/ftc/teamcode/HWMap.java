package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class HWMap {
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor  intake  = null;
    public DcMotor  outtake  = null;

//fyguhigftgyhugftvgy
    public void init(HardwareMap hwMap) {

        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hwMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hwMap.get(DcMotor.class, "backRightDrive");
        intake = hwMap.get(DcMotor.class, "intake");
        outtake = hwMap.get(DcMotor.class, "outtake");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.REVERSE);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Stop();
    }
    public void Stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);
    }

}