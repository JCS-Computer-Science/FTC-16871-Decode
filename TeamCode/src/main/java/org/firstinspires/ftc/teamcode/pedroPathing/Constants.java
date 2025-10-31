package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontL")
            .rightFrontMotorName("frontR")
            .leftRearMotorName("backL")
            .rightRearMotorName("backR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.261926752421046666666666666667)
            .yVelocity(61.494551922189565);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightRear")
            .forwardPodY(6)
            .strafePodX(-7)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
