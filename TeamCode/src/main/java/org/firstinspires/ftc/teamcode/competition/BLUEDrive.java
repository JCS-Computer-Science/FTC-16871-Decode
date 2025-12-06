package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name="BLUE drive", group="competition")
public class BLUEDrive extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    //hardware variables
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shooter = null;
    private DcMotor intake = null;

    //control variables
    public static final double SHOOTER_INTERVAL = 0.1;

    //apriltag/camera variables
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static final double AUTO_TURN = 0.2;


    @Override
    public void runOpMode() {

        initAprilTag();
        String feeder = "Off";
        boolean reversetoggle = false;
        boolean reversed = false;
        //shooter variables
        boolean shooterToggle = false;
        double shooterPower = 0;
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

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //joystick variables
            double max;
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            //auto aim function, points towards certain apriltags within view
            if (gamepad1.a) {
                List<AprilTagDetection> detect = aprilTag.getDetections();
                for (AprilTagDetection dect : detect){
                    if (dect.metadata != null){
                        if (dect.metadata.id == 20){
                            //20 is blue, 24 is red
                            if(dect.ftcPose.x > -0.3){
                                yaw = AUTO_TURN;
                                telemetry.addData("Auto Aim", "Turning right");
                            }else if(dect.ftcPose.x < 0.3){
                                yaw = -AUTO_TURN;
                                telemetry.addData("Auto Aim", "Turning left");
                            }
                        }
                    }else{
                        telemetry.addData("Auto Aim", "No target found");
                    }
                }
            }

            //drive and turning calculations
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            //shooter controls, allows for precise power setting mid match
            if (gamepad1.right_bumper && !shooterToggle && shooterPower < 1) {
                shooterPower += SHOOTER_INTERVAL;
                shooter.setPower(shooterPower);
                shooterToggle = true;
            } else if (gamepad1.left_bumper && !shooterToggle && shooterPower > -0.2) {
                shooterPower -= SHOOTER_INTERVAL;
                shooter.setPower(shooterPower);
                shooterToggle = true;
            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                shooterToggle = false;
            }
            //shooter controls to go right to zero or max
            if (gamepad1.right_trigger > 0.5){
                shooterPower = 1;
                shooter.setPower(shooterPower);
            }
            if (gamepad1.left_trigger > 0.5){
                shooterPower = 0;
                shooter.setPower(shooterPower);
            }

            //intake controls
            if (gamepad1.dpad_down){
                intake.setPower(-0.5);
                feeder = "on";
            }else if (gamepad1.dpad_up){
                intake.setPower(0.5);
                feeder = "reverse";
            }else if (gamepad1.dpad_right){
                intake.setPower(0);
                feeder = "off";
            }else if (gamepad1.dpad_left){
                intake.setPower(1);
                feeder = "reverse 2x";
            };

//            if(gamepad1.b && !reversetoggle) {
//                reversetoggle = true;
//                if(reversed){
//                    frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
//                    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//                    frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
//                    backRightDrive.setDirection(DcMotor.Direction.REVERSE);
//                    reversed = false;
//                }else{
//                    frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//                    backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
//                    frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//                    backRightDrive.setDirection(DcMotor.Direction.FORWARD);
//                    reversed = true;
//                }
//            }else if(!gamepad1.a){
//                reversetoggle = false;
//            }

            //telemetry
            telemetryAprilTag();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Shooter power", shooterPower);
            telemetry.addData("Feeder", feeder);
            telemetry.update();
        }
    }

//    april tag functions
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
