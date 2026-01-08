package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.HWMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveEncoderTest", group = "Debug")
public class DriveEncoderTest extends LinearOpMode {

    HWMap robot = new HWMap();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        DcMotor fl = robot.frontLeftDrive;
        DcMotor fr = robot.frontRightDrive;
        DcMotor bl = robot.backLeftDrive;
        DcMotor br = robot.backRightDrive;

        // Use raw power, no encoder correction
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ENCODER TROUBLESHOOTER READY");
        telemetry.addLine("A=FL | B=FR | X=BL | Y=BR");
        telemetry.addLine("Positive power SHOULD increase encoder count");
        telemetry.update();

        waitForStart();

        int lastFL = fl.getCurrentPosition();
        int lastFR = fr.getCurrentPosition();
        int lastBL = bl.getCurrentPosition();
        int lastBR = br.getCurrentPosition();

        while (opModeIsActive()) {

            // Stop all by default
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            // Drive ONE motor at a time
            if (gamepad1.a) fl.setPower(0.4);
            if (gamepad1.b) fr.setPower(0.4);
            if (gamepad1.x) bl.setPower(0.4);
            if (gamepad1.y) br.setPower(0.4);

            int curFL = fl.getCurrentPosition();
            int curFR = fr.getCurrentPosition();
            int curBL = bl.getCurrentPosition();
            int curBR = br.getCurrentPosition();

            telemetry.addLine("Press and hold ONE button");
            telemetry.addLine("----------------------------");

            telemetry.addData("FL encoder", curFL);
            telemetry.addData("FR encoder", curFR);
            telemetry.addData("BL encoder", curBL);
            telemetry.addData("BR encoder", curBR);

            telemetry.addLine("----------------------------");
            telemetry.addData("ΔFL", curFL - lastFL);
            telemetry.addData("ΔFR", curFR - lastFR);
            telemetry.addData("ΔBL", curBL - lastBL);
            telemetry.addData("ΔBR", curBR - lastBR);

            telemetry.addLine("----------------------------");
            telemetry.addLine("EXPECTED: encoder INCREASES with +power");

            telemetry.update();

            lastFL = curFL;
            lastFR = curFR;
            lastBL = curBL;
            lastBR = curBR;

            sleep(100);
        }
    }
}