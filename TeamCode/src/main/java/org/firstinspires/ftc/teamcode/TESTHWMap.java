package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TESTHWMap {
    public DcMotor Motor1;
    private DcMotor[] motors;

    public void init(HardwareMap hwMap) {
        Motor1  = hwMap.get(DcMotor.class, "Motor1");


        motors = new DcMotor[]{Motor1};

        //directions
        Motor1.setDirection(DcMotor.Direction.FORWARD);


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