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
@Autonomous(name="red Center Stage auto 2+1 far", group="Auto")
public class CenterStageOpenCVAuto2Plus1 extends LinearOpMode {
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
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(0);
                h.motorFrontRight.setTargetPosition(400);
                h.motorBackLeft.setTargetPosition(400);
                h.motorBackRight.setTargetPosition(0);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .3);
                h.motorFrontRight.setPower((float) .3);
                h.motorBackLeft.setPower((float) .3);
                h.motorBackRight.setPower((float) .3);
                while (h.motorFrontRight.getCurrentPosition() < 400 - 20 && h.motorBackLeft.getCurrentPosition() < 400 - 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.drivePureEncoder(true,850, .2);
                h.sleep(50);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(550);
                h.motorFrontRight.setTargetPosition(-550);
                h.motorBackLeft.setTargetPosition(550);
                h.motorBackRight.setTargetPosition(-550);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .3);
                h.motorFrontRight.setPower((float) .3);
                h.motorBackLeft.setPower((float) .3);
                h.motorBackRight.setPower((float) .3);
                while (h.motorBackRight.getCurrentPosition() > -550 + 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() < 550 + 20
                        && h.motorFrontRight.getCurrentPosition() > -550 + 20 && h.motorBackLeft.getCurrentPosition() < 550 - 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(50);
                h.motorIntake.setPower(.4);
                h.sleep(750);
                h.motorIntake.setPower(0);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-550);
                h.motorFrontRight.setTargetPosition(550);
                h.motorBackLeft.setTargetPosition(-550);
                h.motorBackRight.setTargetPosition(550);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .3);
                h.motorFrontRight.setPower((float) .3);
                h.motorBackLeft.setPower((float) .3);
                h.motorBackRight.setPower((float) .3);
                while (h.motorBackRight.getCurrentPosition() < 550 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -550 + 20
                        && h.motorFrontRight.getCurrentPosition() < 550 - 20 && h.motorBackLeft.getCurrentPosition() > -550 + 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(1);
                h.drivePureEncoder(true,1375, .25);
                h.sleep(250);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-1000);
                h.motorFrontRight.setTargetPosition(1000);
                h.motorBackLeft.setTargetPosition(-1000);
                h.motorBackRight.setTargetPosition(1000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() < 1000 - 10 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -1000 + 10
                        && h.motorFrontRight.getCurrentPosition() < 1000 - 10 && h.motorBackLeft.getCurrentPosition() > -1000 + 10) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(500);//keep this sleep
                h.servoIntakeLift.setPosition(.59);
                h.motorLift.setTargetPosition(-475);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                h.drivePureEncoder(true,950, .35);
                h.motorIntake.setPower(-.7);
                sleep(700);
                h.drivePureEncoder(false,200, .35);
                h.servoIntakeLift.setPosition(.76);
                h.motorIntake.setPower(-.7);
                sleep(700);
                h.motorIntake.setPower(0);
                sleep(5);
                h.motorIntake.setPower(.7);
                h.drivePureEncoder(false,3500, .65);
                h.motorIntake.setPower(0);
                h.motorLift.setTargetPosition(-1300);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .6);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-1250);
                h.motorFrontRight.setTargetPosition(1250);
                h.motorBackLeft.setTargetPosition(1250);
                h.motorBackRight.setTargetPosition(-1250);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .3);
                h.motorFrontRight.setPower((float) .3);
                h.motorBackLeft.setPower((float) .3);
                h.motorBackRight.setPower((float) .3);
                while (h.motorBackRight.getCurrentPosition() > -1250 + 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() < 1250 - 20
                        && h.motorFrontRight.getCurrentPosition() < 1250 - 20 && h.motorBackLeft.getCurrentPosition() > -1250 + 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.servoArm.setPosition(.48);
                h.drivePureEncoder(false,1200, .7);
                h.drivePureEncoder(false,300, .2);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-400);
                h.motorFrontRight.setTargetPosition(400);
                h.motorBackLeft.setTargetPosition(400);
                h.motorBackRight.setTargetPosition(-400);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .6);
                h.motorFrontRight.setPower((float) .6);
                h.motorBackLeft.setPower((float) .6);
                h.motorBackRight.setPower((float) .6);
                while (h.motorBackRight.getCurrentPosition() > -400 + 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() < 400 - 20
                        && h.motorFrontRight.getCurrentPosition() < 400 - 20 && h.motorBackLeft.getCurrentPosition() > -400 + 20) {

                }
                h.sleep(1);
                h.servoClaw.setPosition(.55);
                h.sleep(500);
                h.drivePureEncoder(true,200, 1);
                while(h.moveArm(0.21,-1300, 1, 0.24)){}
                sleep(1100);
                while(!h.liftLimit.isPressed()){
                    h.motorLift.setTargetPosition(-30);
                    h.motorLift.setPower(.45);
                }
                break;
            case MIDDLE:
                h.motorLift.setPower(0);
                break;
            case LEFT:
                h.motorLift.setPower(0);
                break;
        }
    }
}
