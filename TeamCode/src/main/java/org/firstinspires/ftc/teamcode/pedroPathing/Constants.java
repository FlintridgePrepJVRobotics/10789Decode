package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
                //.mass(5);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .driveEncoderLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
//sdfsdfsdfsdfsdfsdfs
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            .forwardTicksToInches(0)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .robotWidth(17.5)
            .robotLength(17.5)
            .rightRearEncoderDirection(Encoder.FORWARD);

//https://pedropathing.com/docs/pathing/tuning/localization/drive-encoder
}
