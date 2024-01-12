package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

    int liftBackdropRows[] = new int[]{-2900, -3350, -3805, -4000};
    double extensionBackdropRows[] = new double[]{.47, .65, .66, .67};
    double armBackdropRows[] = new double[]{.3, .31, .321, .322};
    int rowTarget = 1;
    double backdropColumns[] = new double[]{2.5,1.5,0,-1.5,2.5,1.5,0,-1.5,-2.5,1.5,0,-1.5,-2.5};
    double backdropTags[] = new double[]{1,1,1,1,2,2,2,2,2,3,3,3,3};
//    int backdropRow1[] = new int[]{0,1,2,3,4};

    @Override
    public void runOpMode() {
        boolean heightInc = false;
        boolean outtakePressed = false;
        // Initialize hardware class
        Hardware h = new Hardware();
        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            // telemetry.update();
            e.printStackTrace();
        }
        telemetry.addData("outtake encoder value: ", h.servoClaw.getPosition());
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
//
//            if(gamepad2.b) {
//                h.motorFrontLeft.setPower(.7);
//                h.motorFrontRight.setPower(1);
//                h.motorBackLeft.setPower(.7);
//                h.motorBackRight.setPower(1);
//                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            if(gamepad2.right_bumper) {
//                h.motorFrontLeft.setPower(.5);
//                h.motorFrontRight.setPower(-.5);
//                h.motorBackLeft.setPower(-.5);
//                h.motorBackRight.setPower(.5);
//            }

            // Manual lift Controls
            telemetry.addData("arm position: ", h.servoArm.getPosition());
            telemetry.update();
            if(gamepad1.left_bumper && !h.liftLimit.isPressed()){
                moveArmUp = false;//override auto lift
                moveArmDown = false;

                if(h.servoArm.getPosition() > .29){
                    if(rowTarget < 2 && !heightInc) {
                        heightInc = true;
                        rowTarget++;
                    }
                    h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    h.motorLift.setTargetPosition(liftBackdropRows[rowTarget]);
                    h.motorLift.setPower(1);
                    moveArmUp = true;
                } else {
                    h.motorLift.setPower(-.6); //down
                }
            }else if(gamepad1.left_trigger >= 0.05) {
                moveArmUp = false;//override auto lift
                moveArmDown = false;

                if(h.servoArm.getPosition() >.29) {
                    if (rowTarget > 0 && !heightInc) {
                        heightInc = true;
                        rowTarget--;
                    }
                    h.servoArm.setPosition(armBackdropRows[rowTarget]);
                    h.servoExtension.setPosition(extensionBackdropRows[rowTarget]);
                    h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    h.motorLift.setTargetPosition(liftBackdropRows[rowTarget]);
                    h.motorLift.setPower(.5);
                } else {
                    h.motorLift.setPower(gamepad1.left_trigger*.75); //up
                }
            } else {
                heightInc = false;
            }
            if(h.servoArm.getPosition() > .29){ //Math.abs(h.motorLift.getCurrentPosition() - backdropRows[liftTarget]) < 30
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setTargetPosition(liftBackdropRows[rowTarget]);
                h.motorLift.setPower(1);
            }else if(gamepad1.left_trigger < 0.05 && !gamepad1.left_bumper && !moveArmDown && !moveArmUp && !gamepad1.right_bumper){ //if no auto lift and no manual lift, stop motors
                h.motorLift.setPower(0);
            }
//            telemetry.addData("Servo Claw: ", h.servoClaw.getPosition());
//            telemetry.addData("Servo Arm: ", h.servoArm.getPosition());
//            telemetry.addData("Limit Lift: ", h.liftLimit.isPressed());
//            telemetry.addData("Lift Encoder", h.motorLift.getCurrentPosition());
//            telemetry.update();

            //Request to move Swing Arm
            if(gamepad1.dpad_up) {
                h.servoClaw.setPosition(.262);
                rowTarget = 0;
                moveArmUp = true; // Request auto swing up
                moveArmDown = false; // Override swing down
            }
            if(gamepad1.dpad_down) {
                rowTarget = 0;
                moveArmUp = false; // Override swing up
                moveArmDown = true; // swing down
            }

            // Move swing arm
            if(moveArmUp){ // If move up requested
                // Continue to request to move swing arm until the lift is raised to its target or above and the arm has been set to the target position

                moveArmUp = h.moveArm(armBackdropRows[rowTarget], liftBackdropRows[rowTarget], 1.0,extensionBackdropRows[rowTarget]);//.45
            }
            if(moveArmDown){ // If move down requested
                // Continue to request to move swing arm until the lift is raised to -1200 or above and the arm has been set to the target position
                moveArmDown = h.moveArm(0.105, liftBackdropRows[rowTarget], 1.0,.0495);
            }

            // Lift the intake controlse
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
                // Reverse intake (outtake)
                h.motorIntake.setPower(gamepad1.right_trigger + .3);
            } else if(gamepad1.right_bumper) {
                // intake controls
                h.servoClaw.setPosition(.1);
                h.motorIntake.setPower(-1);
            } else {
                h.motorIntake.setPower(0);
            }

            // Drone launching
            if(gamepad2.a) {
                h.servoDrone.setPosition(0);
            } else {
                h.servoDrone.setPosition(1);
            }

            if(gamepad1.b) {
                h.servoExtension.setPosition(.8);
            }
            if(gamepad1.dpad_left) {
                h.servoExtension.setPosition(.30);
            }

//            if(gamepad1.y) {
//                h.servoClaw.setPosition(.225);
//            }

            if(gamepad1.x&&!outtakePressed) {
                if(h.servoClaw.getPosition()>.23){
                    h.servoClaw.setPosition(.225);
                }
                else if (h.servoClaw.getPosition()>.11 && h.servoClaw.getPosition()<.23) {
                    h.servoClaw.setPosition(.1);
                } else {
                    h.servoClaw.setPosition(.225);
                }
                outtakePressed = true;

            }
            if (!gamepad1.x){
                outtakePressed = false;

            }
            if(gamepad1.a) {
                h.servoClaw.setPosition(.262);

            }
        }
    }
}
