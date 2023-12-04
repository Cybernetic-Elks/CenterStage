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
@Autonomous(name="Center Stage auto Blue", group="Auto")
public class CenterStageOpenCVAutoBlue extends LinearOpMode {
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
        PropDetectorBlue detector = new PropDetectorBlue(telemetry);
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

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
//        h.imu = hardwareMap.get(BNO055IMU.class, "imu");
//        h.imu.initialize(parameters);

//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !h.imu.isGyroCalibrated())
//        {
//            sleep(50);
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
            case LEFT:
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(0);
                h.motorFrontRight.setTargetPosition(1500);
                h.motorBackLeft.setTargetPosition(1500);
                h.motorBackRight.setTargetPosition(0);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontRight.getCurrentPosition() < 1500 - 20 && h.motorBackLeft.getCurrentPosition() < 1500 - 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.sleep(2000);
                h.motorIntake.setPower(.8);
                h.sleep(2500);
                h.motorIntake.setPower(0);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(1000);
                h.motorFrontRight.setTargetPosition(-1000);
                h.motorBackLeft.setTargetPosition(1000);
                h.motorBackRight.setTargetPosition(-1000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() > -1000 + 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() < 1000 - 20
                        && h.motorFrontRight.getCurrentPosition() > -1000 + 20 && h.motorBackLeft.getCurrentPosition() < 1000 - 20) {

                }
                h.sleep(500);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setTargetPosition(-1485);//-145
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                while (h.motorLift.getCurrentPosition() > -1485 + 20 && !isStopRequested()){}
                h.sleep(1000);
                h.servoArm.setPosition(.45);
                h.drivePureEncoder(false, 1315, .2); 
                h.sleep(500);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-100);
                h.motorFrontRight.setTargetPosition(100);
                h.motorBackLeft.setTargetPosition(100);
                h.motorBackRight.setTargetPosition(-100);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontRight.getCurrentPosition() < 100 - 20 && h.motorBackLeft.getCurrentPosition() < 100 - 20 &&
                        h.motorFrontLeft.getCurrentPosition() > -100 + 20 && h.motorBackRight.getCurrentPosition() > -100 + 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(500);
                h.servoClaw.setPosition(.43);
                h.sleep(500);
                h.drivePureEncoder(true, 100, .2);
                h.sleep(250);
                h.servoArm.setPosition(0.22);
                h.motorLift.setTargetPosition(-30);
                break;
            case MIDDLE:
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(0);
                h.motorFrontRight.setTargetPosition(300);
                h.motorBackLeft.setTargetPosition(300);
                h.motorBackRight.setTargetPosition(0);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontRight.getCurrentPosition() < 300 - 20 && h.motorBackLeft.getCurrentPosition() < 300 - 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.sleep(500);
                h.drivePureEncoder(true, 1400, .2);
                sleep(500);
                h.drivePureEncoder(false, 400, .2);
                h.sleep(1000);
                h.motorIntake.setPower(.6);
                h.sleep(2500);
                h.motorIntake.setPower(0);
                h.sleep(1000);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(1000);
                h.motorFrontRight.setTargetPosition(-1000);
                h.motorBackLeft.setTargetPosition(1000);
                h.motorBackRight.setTargetPosition(-1000);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackLeft.getCurrentPosition() < 1000 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() < 1000 - 20
                        && h.motorFrontRight.getCurrentPosition() > -1000 + 20 && h.motorBackRight.getCurrentPosition() > -1000 + 20) {

                }
                h.motorLift.setTargetPosition(-1350);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                while (h.motorLift.getCurrentPosition() > -1350 + 20 && !isStopRequested()){}
                h.sleep(500);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.servoArm.setPosition(.45);
                h.drivePureEncoder(false, 1940, .2);
                h.sleep(500);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-150);
                h.motorFrontRight.setTargetPosition(150);
                h.motorBackLeft.setTargetPosition(150);
                h.motorBackRight.setTargetPosition(-150);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontRight.getCurrentPosition() < 150 - 20 && h.motorBackLeft.getCurrentPosition() < 150 - 20 &&
                        h.motorFrontLeft.getCurrentPosition() > -150 + 20 && h.motorBackRight.getCurrentPosition() > -150 + 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(500);
                h.servoClaw.setPosition(.43);
                h.sleep(500);
                h.drivePureEncoder(true, 100, .2);
                h.sleep(250);
                h.servoArm.setPosition(0.22);
                h.motorLift.setTargetPosition(-30);
                break;
            case RIGHT:
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(0);
                h.motorFrontRight.setTargetPosition(500);
                h.motorBackLeft.setTargetPosition(500);
                h.motorBackRight.setTargetPosition(0);;

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontRight.getCurrentPosition() < 500 - 20 && h.motorBackLeft.getCurrentPosition() < 500 - 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.sleep(500);
                h.drivePureEncoder(true, 1250, .2);
                h.sleep(1000);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(1020);
                h.motorFrontRight.setTargetPosition(-1020);
                h.motorBackLeft.setTargetPosition(1020);
                h.motorBackRight.setTargetPosition(-1020);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() > -1020 + 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() < 1020 - 20
                        && h.motorFrontRight.getCurrentPosition() > -1020 + 20 && h.motorBackLeft.getCurrentPosition() < 1020 - 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.drivePureEncoder(true, 180, .2);
                h.drivePureEncoder(false, 180, .2);

                h.sleep(2000);
                h.motorIntake.setPower(.7);
                h.sleep(2000);
                h.motorIntake.setPower(0);
                h.sleep(2000);
                h.motorLift.setTargetPosition(-1350);//-145
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                while (h.motorLift.getCurrentPosition() > -1350 + 20 && !isStopRequested()){}
                h.sleep(1000);
                h.servoArm.setPosition(.45);
                h.drivePureEncoder(false, 1840, .2);
                h.sleep(1000);
                h.motorLift.setTargetPosition(-1100);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                while (h.motorLift.getCurrentPosition() > -1350 + 20 && !isStopRequested()){}
                h.sleep(3000);
                h.servoClaw.setPosition(.43);
                h.sleep(500);
                h.drivePureEncoder(true, 100, .2);
                h.sleep(250);
                h.servoArm.setPosition(0.22);
                h.motorLift.setTargetPosition(-30);
                break;
        }
    }
}
