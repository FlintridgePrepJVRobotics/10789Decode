//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "TESTTESTDECODE")
//public class TESTING extends LinearOpMode {
//    public HWMap robot = new HWMap();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//
//        waitForStart();
//        double speed = 1;
//
//        while (opModeIsActive()) {
//            if (gamepad1.a) {//intake
//                robot.flywheelOne.setPower(1);
//                robot.flywheelTwo.setPower(1);
//            }
//            else{
//                robot.flywheelOne.setPower(0);
//            }
//
//            if (gamepad1.b) {
//                robot.flywheelOne.setPower(1);
//                robot.flywheelTwo.setPower(1);
//            }
//            if (gamepad1.x){
//                robot.flywheelOne.setPower(0);
//                robot.flywheelTwo.setPower(0);
//            }
//
//        }
//    }
//}
