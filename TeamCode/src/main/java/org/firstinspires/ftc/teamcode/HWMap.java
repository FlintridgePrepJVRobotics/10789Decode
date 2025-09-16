package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class HWMap {
    public DcMotor  LeftDrive   = null;
    public DcMotor  RightDrive  = null;

    public void init(HardwareMap hwMap) {

        LeftDrive  = hwMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hwMap.get(DcMotor.class, "RightDrive");

        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);

        RightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Stop();
//dfgjkdfgjkdfghjkasdasd
    }
    public void Stop() {
        RightDrive.setPower(0);
        LeftDrive.setPower(0);
    }

}
//jhnbg