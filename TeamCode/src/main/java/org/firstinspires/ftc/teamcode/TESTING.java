package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TESTTESTDECODE")
public class TESTING extends LinearOpMode {
    public TESTHWMap robot = new TESTHWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        double speed = 1;

        while (opModeIsActive()) {
            if (gamepad1.a) {//intake
                robot.Motor1.setPower(1);
            }
        }
    }
}
