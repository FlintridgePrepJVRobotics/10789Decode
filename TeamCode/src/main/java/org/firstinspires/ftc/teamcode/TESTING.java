package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TESTTESTDECODE")
public class TESTING extends LinearOpMode {
    public FWHWMap flywheel = new FWHWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel.init(hardwareMap);

        waitForStart();
        double speed = 1;

        while (opModeIsActive()) {
            if (gamepad1.a) {//intake
                flywheel.flywheelOne.setPower(1);
//                flywheel.flywheelTwo.setPower(1);
            }
            else{
                flywheel.flywheelOne.setPower(0);
            }

            if (gamepad1.b) {
                flywheel.flywheelOne.setPower(-1);
            }
            else{
                flywheel.flywheelOne.setPower(0);
            }

        }
    }
}
