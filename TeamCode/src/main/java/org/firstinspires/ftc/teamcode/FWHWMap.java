package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FWHWMap {
    public DcMotor flywheelOne, flywheelTwo;
    private DcMotor[] motors;

    public void init(HardwareMap flywheelHWMap) {
        flywheelOne  = flywheelHWMap.get(DcMotor.class, "flywheelOne");
        flywheelTwo  = flywheelHWMap.get(DcMotor.class, "flywheelTwo");
//        feedServo = flywheelWHWMap.get(Servo.class, "feedServo");


        motors = new DcMotor[]{flywheelOne, flywheelTwo};

        //directions
        flywheelOne.setDirection(DcMotor.Direction.FORWARD);
        flywheelTwo.setDirection(DcMotor.Direction.FORWARD);


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