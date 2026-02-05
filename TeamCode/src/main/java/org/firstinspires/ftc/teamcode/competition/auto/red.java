package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "RED AUTO", group = "competitive")
public class red extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //hardware variables
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shooter = null;
    private DcMotor intake = null;
    private static final boolean USE_WEBCAM = true;


    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;
    private static final int DESIRED_TAG_ID = 20;
    private static final double AUTO_TURN = 0.15;

    private int checks = 0;


    @Override
    public void runOpMode(){
        boolean targetFound = false;
        //hardware assigning, make sure device names in here match the ones in config
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontL");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontR");
        backRightDrive = hardwareMap.get(DcMotor.class, "backR");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //directions of wheels, may need to change directions to drive properly
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //direction of shooter
        shooter.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initAprilTag();
        waitForStart();
        runtime.reset();

        shooter.setPower(0.85);
        runtime.reset();

        while (opModeIsActive() && !targetFound) {
            List<AprilTagDetection> detect = aprilTag.getDetections();
            for (AprilTagDetection dect : detect){
                if (dect.metadata != null) {
                    if (dect.metadata.id == DESIRED_TAG_ID) {
                        //20 is blue, 24 is red
                        checks++;
                        if (dect.ftcPose.x > 9.5) {
                            frontLeftDrive.setPower(AUTO_TURN);
                            frontRightDrive.setPower(-AUTO_TURN);
                            backLeftDrive.setPower(AUTO_TURN);
                            backRightDrive.setPower(-AUTO_TURN);
                            telemetry.addData("Auto Aim", "Turning right");
                            telemetry.addData("Checks", checks);
                            telemetryAprilTag();
                            telemetry.update();
                        }
                        if (dect.ftcPose.x < 6) {
                            frontLeftDrive.setPower(-AUTO_TURN);
                            frontRightDrive.setPower(AUTO_TURN);
                            backLeftDrive.setPower(-AUTO_TURN);
                            backRightDrive.setPower(AUTO_TURN);
                            telemetry.addData("Auto Aim", "Turning left");
                            telemetry.addData("Checks", checks);
                            telemetryAprilTag();
                            telemetry.update();
                        }
                        if (dect.ftcPose.x > 6&& dect.ftcPose.x < 9.5){
                            targetFound = true;
                            frontLeftDrive.setPower(0);
                            frontRightDrive.setPower(0);
                            backLeftDrive.setPower(0);
                            backRightDrive.setPower(0);
                        }
                    }
                }
                telemetry.addData("Auto Aim", "No target found");
                telemetry.update();
            }
            telemetry.addData("Finished", "AAAAAAAAAAA");
            telemetry.update();
        }

        intake.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 7)) {
            telemetry.addData("shooting", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        shooter.setPower(0);
        intake.setPower(0);
        frontLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("moving", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()

                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }
}