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
 * Date Created:  
 * Purpose: This is going to be our main teleop for CenterStage
 */

public class TeleOp2024 extends LinearOpMode {
    OpMode opmode;

    // booleans for auto lift
    boolean pressed = false;
    boolean moveArmUp = false;
    boolean moveArmDown = false;
    int liftTarget = -1200;

    @Override
    public void runOpMode() {
        // Initialize hardware class
        Hardware h = new Hardware();
        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            // telemetry.update();
            e.printStackTrace();
        }

        //initialize AprilTag class
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
//        while (!isStopRequested() && !h.imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//        }

//                telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {

            //Ability to drive
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false, 2, 1);

            //AprilTags for goodness
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if (gamepad2.a) {
                    if (tag.ftcPose.yaw > 2) {
                        h.turnnoIMU(true, (int)Math.abs(tag.ftcPose.yaw)*10+20, .1);//20
                    } else if (tag.ftcPose.yaw < -2) {
                        h.turnnoIMU(false, (int)Math.abs(tag.ftcPose.yaw)*10+20, .1);
                    } else if (tag.ftcPose.x < -2) {
                        h.strafePureEncoder(true, (int)Math.abs(tag.ftcPose.x)*40+20, .2);
                    } else if (tag.ftcPose.x > 2) {
                        h.strafePureEncoder(false, (int)Math.abs(tag.ftcPose.x)*40+20, .2);
                    } else if (tag.ftcPose.y > 2) {
                        h.drivePureEncoder(false, (int)Math.abs(tag.ftcPose.y)+20, .2);
                    }
                }
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("tag", tag.id);
                telemetry.addData("motorfrontleft", h.motorFrontLeft.getCurrentPosition());
                telemetry.addData("motorfrontright", h.motorFrontRight.getCurrentPosition());
                telemetry.addData("motorbackleft", h.motorBackLeft.getCurrentPosition());
                telemetry.addData("motorbackright", h.motorBackRight.getCurrentPosition());
                telemetry.update();
            }
            if(gamepad2.b) {
                h.motorFrontLeft.setPower(.7);
                h.motorFrontRight.setPower(1);
                h.motorBackLeft.setPower(.7);
                h.motorBackRight.setPower(1);
                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad2.right_bumper) {
                h.motorFrontLeft.setPower(.5);
                h.motorFrontRight.setPower(-.5);
                h.motorBackLeft.setPower(-.5);
                h.motorBackRight.setPower(.5);
            }

            // Manual lift Controls
            if(gamepad1.left_trigger >= 0.05 && !h.liftLimit.isPressed()){
                moveArmUp = false;//override auto lift
                moveArmDown = false;
                h.motorLift.setPower(gamepad1.left_trigger*.75); //down
            }if(gamepad1.left_bumper) {
                moveArmUp = false;//override auto lift
                moveArmDown = false;
                h.motorLift.setPower(-.6); //up
            }else if(gamepad1.left_trigger < 0.05 && !gamepad1.left_bumper && !moveArmDown && !moveArmUp){ //if no auto lift and no manual lift, stop motors
                h.motorLift.setPower(0);
            }

            // Claw controls
            if(gamepad1.x){ //Grab
                h.servoClaw.setPosition(0.01);//00.1
            }
            if(gamepad1.y){ //Release
                h.servoClaw.setPosition(.6);//.6
            }
            telemetry.addData("Servo Claw: ", h.servoClaw.getPosition());
            telemetry.addData("Servo Arm: ", h.servoArm.getPosition());
            telemetry.addData("Limit Lift: ", h.liftLimit.isPressed());
            telemetry.addData("Lift Encoder", h.motorLift.getCurrentPosition());
            telemetry.update();

            //Request to move Swing Arm
            if(gamepad1.dpad_up) {
                moveArmUp = true; // Request auto swing up
                moveArmDown = false; // Override swing down
            }
            if(gamepad1.dpad_down) {
                moveArmUp = false; // Override swing up
                moveArmDown = true; // swing down
            }

            // Move swing arm
            if(moveArmUp){ // If move up requested
                // Continue to request to move swing arm until the lift is raised to its target or above and the arm has been set to the target position
                moveArmUp = h.moveArm(0.45, liftTarget, 1.0);
            }
            if(moveArmDown){ // If move down requested
                // Continue to request to move swing arm until the lift is raised to -1200 or above and the arm has been set to the target position
                moveArmDown = h.moveArm(0.2, -1200, 1.0);
            }

            // Lift the intake controls
            if(gamepad2.dpad_down){
                // Lower intake to flow (1 pixel high)
                h.servoIntakeLift.setPosition(.76);//.76
            }
            if(gamepad2.dpad_right){
                // Raise/Lower intake to middle height (3 pixel high)
                h.servoIntakeLift.setPosition(.72);//.77
            }
            if(gamepad2.dpad_up){
                // Raise intake to highest height (5 pixels high)
                h.servoIntakeLift.setPosition(.67);//.67
            }

            // Intake controls
            if(gamepad1.right_trigger>.10) {
                // Intake pixel
                h.motorIntake.setPower(gamepad1.right_trigger + .3);
            } else if(gamepad1.right_bumper) {
                // Reverse intake (outtake)
                h.motorIntake.setPower(-.7);
            } else {
                // Turn intake off when not in use
                h.motorIntake.setPower(0);
            }
        }
    }
}
