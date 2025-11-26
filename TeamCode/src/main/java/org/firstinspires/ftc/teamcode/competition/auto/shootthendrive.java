package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class shootthendrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //hardware variables
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shooter = null;
    private DcMotor intake = null;

    @Override
    public void runOpMode(){
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
    }
}
