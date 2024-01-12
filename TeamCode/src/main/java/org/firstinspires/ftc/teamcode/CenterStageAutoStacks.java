package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@Autonomous(name="Center Stage auto red 2+2", group="Auto")
public class CenterStageAutoStacks extends LinearOpMode {
    Hardware h = new Hardware();
    OpenCvCamera webCam;
    int driveDiagonal;
    int driveDiagonal2;
    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT
    }
    Position position;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "intakeWebcam"), cameraMonitorViewId);
        PropDetector detector = new PropDetector(telemetry);
        webCam.setPipeline(detector);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error", "Camera not able to open");
            }
        });
        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        h.imu = hardwareMap.get(BNO055IMU.class, "imu");
//        h.imu.initialize(parameters);

//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !h.imu.isGyroCalibrated())
//        {
            sleep(50);
//            idle();
//        }

//        telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
//        telemetry.update();

        //h.servoIntake.setPosition(1);

        waitForStart();
        switch (detector.getLocation()) {
            case LEFT: //bottom reversed if blue
                position = Position.LEFT;
                break;
            case MIDDLE://middle reversed if blue
                position = Position.MIDDLE;
                break;
            case RIGHT://top reversed if blue
                position = Position.RIGHT;
        }
        webCam.stopStreaming();

        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        h.servoIntakeLift.setPosition(.76);

        switch (position) {
            case RIGHT:
                h.driveDiagonal(true, 1500, .5);
                h.sleep(5);
                h.motorIntake.setPower(.6);
                h.sleep(750);
                h.motorIntake.setPower(0);
                h.motorLift.setTargetPosition(-1485);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .4);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-975);
                h.motorFrontRight.setTargetPosition(975);
                h.motorBackLeft.setTargetPosition(-975);
                h.motorBackRight.setTargetPosition(975);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .5);
                h.motorFrontRight.setPower((float) .5);
                h.motorBackLeft.setPower((float) .5);
                h.motorBackRight.setPower((float) .5);
                //this is to tell the robot to stop the slides which started move at the beginning
                while (h.motorLift.getCurrentPosition() > -1485 + 20 && !isStopRequested()){}
                while (h.motorBackRight.getCurrentPosition() < 975 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -975 + 20
                        && h.motorFrontRight.getCurrentPosition() < 975 - 20 && h.motorBackLeft.getCurrentPosition() > -975 + 20) {
                }
                h.sleep(1);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(250);
                h.servoArm.setPosition(.45);
                h.drivePureEncoder(false, 825, .5);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(0);
                h.motorFrontRight.setTargetPosition(-865);
                h.motorBackLeft.setTargetPosition(-865);
                h.motorBackRight.setTargetPosition(0);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .5);
                h.motorFrontRight.setPower((float) .5);
                h.motorBackLeft.setPower((float) .5);
                h.motorBackRight.setPower((float) .5);
                while (h.motorFrontLeft.getCurrentPosition() > -865 + 20 && h.motorBackLeft.getCurrentPosition() > -865 + 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(250);
                h.drivePureEncoder(false, 525, .35);
                h.sleep(5);
                h.servoClaw.setPosition(.43);
                h.sleep(500);
                h.motorLift.setTargetPosition(-250);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .3);
                while (h.motorLift.getCurrentPosition() > -250 + 20 && !isStopRequested()){}
                h.sleep(50);
                while(h.moveArm(0.20,0, 1, 0.24)){}
                h.drivePureEncoder(true, 100, .35);
                sleep(1750);
                h.sleep(250);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(0);
                h.motorFrontRight.setTargetPosition(865);
                h.motorBackLeft.setTargetPosition(865);
                h.motorBackRight.setTargetPosition(0);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .5);
                h.motorFrontRight.setPower((float) .5);
                h.motorBackLeft.setPower((float) .5);
                h.motorBackRight.setPower((float) .5);
                while (h.motorFrontLeft.getCurrentPosition() < 850 - 20 && h.motorBackLeft.getCurrentPosition() < 850 - 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(50);
                h.drivePureEncoder(true, 1000, .1);
                while(!h.liftLimit.isPressed()){
                    h.motorLift.setTargetPosition(-30);
                    h.motorLift.setPower(.4);
                }
                h.motorLift.setPower(0);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(750);
                h.motorFrontRight.setTargetPosition(-750);
                h.motorBackLeft.setTargetPosition(-750);
                h.motorBackRight.setTargetPosition(750);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .3);
                h.motorFrontRight.setPower((float) .3);
                h.motorBackLeft.setPower((float) .3);
                h.motorBackRight.setPower((float) .3);
                while (h.motorFrontLeft.getCurrentPosition() > -750 + 20 && h.motorBackLeft.getCurrentPosition() > -750 + 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(10);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-50);
                h.motorFrontRight.setTargetPosition(50);
                h.motorBackLeft.setTargetPosition(-50);
                h.motorBackRight.setTargetPosition(50);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .3);
                h.motorFrontRight.setPower((float) .3);
                h.motorBackLeft.setPower((float) .3);
                h.motorBackRight.setPower((float) .3);
                while (h.motorFrontLeft.getCurrentPosition() > -50 + 20 && h.motorBackLeft.getCurrentPosition() > -50 + 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorIntake.setPower(-.8);
                h.drivePureEncoder(true, 3500, .1);
                break;
//            case MIDDLE:
//                h.driveDiagonal(true, 300, .2);
//                h.sleep(500);
//                h.drivePureEncoder(true, 1200, .2);
//                sleep(500);
//                h.drivePureEncoder(false, 250, .2);
//                h.sleep(50);
//                h.motorIntake.setPower(.4);
//                h.sleep(1500);
//                h.motorIntake.setPower(0);
//                h.sleep(1000);
//                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                h.motorFrontLeft.setTargetPosition(-1000);
//                h.motorFrontRight.setTargetPosition(1000);
//                h.motorBackLeft.setTargetPosition(-1000);
//                h.motorBackRight.setTargetPosition(1000);
//
//                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                h.motorFrontLeft.setPower((float) .2);
//                h.motorFrontRight.setPower((float) .2);
//                h.motorBackLeft.setPower((float) .2);
//                h.motorBackRight.setPower((float) .2);
//                while (h.motorBackRight.getCurrentPosition() < 1000 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -1000 + 20
//                        && h.motorFrontRight.getCurrentPosition() < 1000 - 20 && h.motorBackLeft.getCurrentPosition() > -1000 + 20) {
//
//                }
//                h.motorLift.setTargetPosition(-1350);
//                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorLift.setPower((float) .2);
//                while (h.motorLift.getCurrentPosition() > -1350 + 20 && !isStopRequested()){}
//                h.sleep(500);
//                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.servoArm.setPosition(.45);
//                h.drivePureEncoder(false, 1940, .2); // Josiah changed from 1625 to 1940
//                h.sleep(500);
//                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                h.motorFrontLeft.setTargetPosition(300);
//                h.motorFrontRight.setTargetPosition(-300);
//                h.motorBackLeft.setTargetPosition(-300);
//                h.motorBackRight.setTargetPosition(300);
//
//                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                h.motorFrontLeft.setPower((float) .2);
//                h.motorFrontRight.setPower((float) .2);
//                h.motorBackLeft.setPower((float) .2);
//                h.motorBackRight.setPower((float) .2);
//                while (h.motorFrontRight.getCurrentPosition() > -300 + 20 && h.motorBackLeft.getCurrentPosition() > -300 + 20 &&
//                        h.motorFrontLeft.getCurrentPosition() < 300 - 20 && h.motorBackRight.getCurrentPosition() < 300 - 20) {}
//                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.sleep(500);
//                h.servoClaw.setPosition(.43);
//                h.sleep(500);
//                h.drivePureEncoder(true, 100, .2);
//                h.sleep(500);
//                while(h.moveArm(0.21,-1300, 1)){}
//                sleep(1750);
//                while(!h.liftLimit.isPressed()){
//                    h.motorLift.setTargetPosition(-30);
//                    h.motorLift.setPower(.15);
//                }
//                h.motorLift.setPower(0);
//                break;
//            case LEFT:
//                h.driveDiagonal(true, 500, .2);
//                h.sleep(500);
//                h.drivePureEncoder(true, 1250, .2);
//                h.sleep(1000);
//                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                h.motorFrontLeft.setTargetPosition(-1020);
//                h.motorFrontRight.setTargetPosition(1020);
//                h.motorBackLeft.setTargetPosition(-1020);
//                h.motorBackRight.setTargetPosition(1020);
//
//                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                h.motorFrontLeft.setPower((float) .2);
//                h.motorFrontRight.setPower((float) .2);
//                h.motorBackLeft.setPower((float) .2);
//                h.motorBackRight.setPower((float) .2);
//                while (h.motorBackRight.getCurrentPosition() < 1020 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -1020 + 20
//                        && h.motorFrontRight.getCurrentPosition() < 1020 - 20 && h.motorBackLeft.getCurrentPosition() > -1020 + 20) {
//
//                }
//                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                h.drivePureEncoder(true, 180, .2);
//                h.drivePureEncoder(false, 180, .2);
//
//                h.sleep(750);
//                h.motorIntake.setPower(.4);
//                h.sleep(1250);
//                h.motorIntake.setPower(0);
//                h.sleep(1);
//                h.motorLift.setTargetPosition(-1350);//-145
//                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                h.motorLift.setPower((float) .2);
//                while (h.motorLift.getCurrentPosition() > -1350 + 20 && !isStopRequested()){}
//                h.sleep(500);
//                h.servoArm.setPosition(.45);
//                h.drivePureEncoder(false, 1840, .2); // Josiah changed from 1525 to 1840
//                h.sleep(500);
//                h.servoClaw.setPosition(.43);
//                h.sleep(500);
//                h.drivePureEncoder(true, 100, .2);
//                h.sleep(500);
//                while(h.moveArm(0.21,-1300, 1)){}
//                sleep(1750);
//                while(!h.liftLimit.isPressed()){
//                    h.motorLift.setTargetPosition(-30);
//                    h.motorLift.setPower(.15);
//                }
//                h.motorLift.setPower(0);
//                break;
        }
    }
}
