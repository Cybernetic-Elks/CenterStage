package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "2024 TeleOp - CHOOSE THIS ONE", group = "TeleOp")
/**
 * Date Created:  7/30/2022
 * Purpose: This is going to be our main teleop for PowerPlay
 */

public class TeleOp2024 extends LinearOpMode {
    OpMode opmode;
    boolean pressed = false;

    @Override
    public void runOpMode() {
        Hardware h = new Hardware();
        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(hardwareMap.get(WebcamName.class, "outtakeWebcam"))
                    .build();

        // IMU not working yet, saving this for later
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        h.imu = hardwareMap.get(BNO055IMU.class, "imu");
//        h.imu.initialize(parameters);
//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !h.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();


        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();


        waitForStart();
        while (opModeIsActive()) {
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false, 2, 1);

            //AprilTags for goodness
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if (gamepad2.a) {
                    if (tag.ftcPose.yaw > 0) {
                        h.turnnoIMU(true, 50, .2);
                    }
                    if (tag.ftcPose.yaw < 0) {
                        h.turnnoIMU(false, 50, .2);
                    }
                }
            }

            //all off the inputs
            if(gamepad1.left_trigger > 0.05){
                h.motorLift.setPower(gamepad1.left_trigger*.75);
            }else{
                h.motorLift.setPower(0);
            }
            if(gamepad1.x && pressed == false){
                pressed=true;
                h.servoClaw.setPosition(.43);//h.servoClaw.getPosition()+.01
            }
            if(gamepad1.y && pressed == false){
                pressed=true;
                h.servoClaw.setPosition(.1);//h.servoClaw.getPosition()-.01
            }
            if(gamepad1.dpad_up && pressed == false){
                pressed = true;
                h.servoArm.setPosition(.45);//.45
            }
            if(gamepad2.dpad_down){
                h.servoIntakeLift.setPosition(.76);//h.servoIntakeLift.getPosition()-.01
            }
            if(!gamepad1.x&&!gamepad1.y &&!gamepad1.dpad_up &&!gamepad1.dpad_down){
                pressed = false;
            }
            if(gamepad2.dpad_right){
                h.servoIntakeLift.setPosition(.72);//.77
            }
            if(gamepad1.right_trigger>.10) {
                h.motorIntake.setPower(gamepad1.right_trigger + .3);
            }
            if(gamepad1.b) {
                h.motorIntake.setPower(0);
            }
            if(gamepad1.left_bumper) {
                h.motorLift.setPower(-.6);
            }
            if(gamepad1.right_bumper) {
                h.motorIntake.setPower(-.7);
            }
        }
    }
}
