package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HWMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="encoders")
//"tag" that is displayed on driver hub
public class encoders extends LinearOpMode {
    //creating robot object
    //the project name will be different, make sure to change this line below to proper name and also in the imports
    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //wait for start button to be pressed
        waitForStart();

        //write autonomous code here

        //record number of encoder counts for certain distances
        //"blank" encoder counts = 1 tile
        //"blank" encoder counts = 90 degree turn

        //speed, leftCounts, rightCounts
        encoderDrive (0.5, 1200, 1200, 1200, 1200);
        encoderDrive (0.5, 200, -200, 200, -200);
        robot.flywheelOne.setPower(1);
        robot.flywheelTwo.setPower(1);
        sleep(3000);
        robot.feedServo.setPosition(1);
        robot.feedServo.setPosition(0);
        robot.flywheelOne.setPower(0);
        robot.flywheelTwo.setPower(0);
        robot.feedServo.setPosition(0);


    }


    //encoder method
    public void encoderDrive(double speed,
                             double fRightCounts, double fLeftCounts, double bRightCounts, double bLeftCounts) {
        int newfLeftTarget;
        int newfRightTarget;
        int newbLeftTarget;
        int newbRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (fLeftCounts);
            newfRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (fRightCounts);
            newbLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int) (bLeftCounts);
            newbRightTarget = robot.backRightDrive.getCurrentPosition() + (int) (bRightCounts);
            robot.frontLeftDrive.setTargetPosition(newfLeftTarget);
            robot.frontRightDrive.setTargetPosition(newfRightTarget);
            robot.backLeftDrive.setTargetPosition(newbLeftTarget);
            robot.backRightDrive.setTargetPosition(newbRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newfLeftTarget, newfRightTarget, newbLeftTarget, newbRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition(),
                        robot.backLeftDrive.getCurrentPosition(),
                        robot.backRightDrive.getCurrentPosition());


                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}