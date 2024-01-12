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
@Autonomous(name="Center Stage Auto 1+1", group="Auto")
public class CenterStageAuto extends LinearOpMode {
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
                h.driveDiagonal(true, 1500, .4);
                h.sleep(250);
                h.motorIntake.setPower(.6);
                h.sleep(750);
                h.motorIntake.setPower(0);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-975);
                h.motorFrontRight.setTargetPosition(0);
                h.motorBackLeft.setTargetPosition(0);
                h.motorBackRight.setTargetPosition(-975);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontLeft.getCurrentPosition() > -1000 + 20 && h.motorBackRight.getCurrentPosition() > -1000 + 20) {

                }
                sleep(250);
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

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() < 975 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -975 + 20
                        && h.motorFrontRight.getCurrentPosition() < 975 - 20 && h.motorBackLeft.getCurrentPosition() > -975 + 20) {

                }
                h.sleep(250);
                h.motorLift.setTargetPosition(-475);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                while (h.motorLift.getCurrentPosition() > -475 + 20 && !isStopRequested()){}
                h.drivePureEncoder(true,1225,.4);
                h.sleep(50);
                h.motorIntake.setPower(-.9);
                h.sleep(500);
                h.drivePureEncoder(false,600,.4);
                h.sleep(1);
                h.drivePureEncoder(false,2000,.4);
                h.motorLift.setTargetPosition(-1485);//-145
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                while (h.motorLift.getCurrentPosition() > -1485 + 20 && !isStopRequested()){}
                h.sleep(500);
                h.motorIntake.setPower(0);
                h.servoArm.setPosition(.45);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(800);
                h.motorFrontRight.setTargetPosition(-800);
                h.motorBackLeft.setTargetPosition(-800);
                h.motorBackRight.setTargetPosition(800);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .15);
                h.motorFrontRight.setPower((float) .15);
                h.motorBackLeft.setPower((float) .15);
                h.motorBackRight.setPower((float) .15);
                while (h.motorFrontRight.getCurrentPosition() > -800 + 20 && h.motorBackLeft.getCurrentPosition() > -800 + 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(1250);
                h.drivePureEncoder(false, 650, .2);
                h.sleep(50);
                h.servoClaw.setPosition(.43);
                h.sleep(1000);
                h.drivePureEncoder(true, 100, .2);
                h.sleep(1);
                while(h.moveArm(0.21,-1300, 1, 0.24)){}
                sleep(1750);
                while(!h.liftLimit.isPressed()){
                    h.motorLift.setTargetPosition(-30);
                    h.motorLift.setPower(.3);
                }
                h.motorLift.setPower(0);
                break;
            case MIDDLE:
                h.driveDiagonal(true, 300, .2);
                h.sleep(500);
                h.drivePureEncoder(true, 1200, .2);
                sleep(500);
                h.drivePureEncoder(false, 250, .2);
                h.sleep(50);
                h.motorIntake.setPower(.4);
                h.sleep(1500);
                h.motorIntake.setPower(0);
                h.sleep(1000);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-700);
                h.motorFrontRight.setTargetPosition(700);
                h.motorBackLeft.setTargetPosition(-700);
                h.motorBackRight.setTargetPosition(700);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() < 700 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -700 + 20
                        && h.motorFrontRight.getCurrentPosition() < 700 - 20 && h.motorBackLeft.getCurrentPosition() > -700 + 20) {

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
                h.drivePureEncoder(false, 1940, .2); // Josiah changed from 1625 to 1940
                h.sleep(500);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(300);
                h.motorFrontRight.setTargetPosition(-300);
                h.motorBackLeft.setTargetPosition(-300);
                h.motorBackRight.setTargetPosition(300);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorFrontRight.getCurrentPosition() > -300 + 20 && h.motorBackLeft.getCurrentPosition() > -300 + 20 &&
                        h.motorFrontLeft.getCurrentPosition() < 300 - 20 && h.motorBackRight.getCurrentPosition() < 300 - 20) {}
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(500);
                h.servoClaw.setPosition(.43);
                h.sleep(500);
                h.drivePureEncoder(true, 100, .2);
                h.sleep(1);
                while(h.moveArm(0.21,-1300, 1, .24)){}
                sleep(1750);
                while(!h.liftLimit.isPressed()){
                    h.motorLift.setTargetPosition(-30);
                    h.motorLift.setPower(.15);
                }
                h.motorLift.setPower(0);
                break;
            case LEFT:
                h.driveDiagonal(true, 500, .2);
                h.sleep(25);
                h.drivePureEncoder(true, 1250, .2);
                h.sleep(250);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-1020);
                h.motorFrontRight.setTargetPosition(1020);
                h.motorBackLeft.setTargetPosition(-1020);
                h.motorBackRight.setTargetPosition(1020);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() < 1020 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -1020 + 20
                        && h.motorFrontRight.getCurrentPosition() < 1020 - 20 && h.motorBackLeft.getCurrentPosition() > -1020 + 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.drivePureEncoder(true, 180, .2);
                h.drivePureEncoder(false, 180, .2);

                h.sleep(250);
                h.motorLift.setTargetPosition(-475);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower((float) .2);
                h.motorIntake.setPower(.4);
                h.sleep(750);
                h.motorIntake.setPower(0);
                while (h.motorLift.getCurrentPosition() > -475 + 20 && !isStopRequested()){}
                h.sleep(500);
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
                while (h.motorBackRight.getCurrentPosition() < 1020 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -1020 + 20
                        && h.motorFrontRight.getCurrentPosition() < 1020 - 20 && h.motorBackLeft.getCurrentPosition() > -1020 + 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.drivePureEncoder(false, 1250, .2);
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(-1020);
                h.motorFrontRight.setTargetPosition(1020);
                h.motorBackLeft.setTargetPosition(-1020);
                h.motorBackRight.setTargetPosition(1020);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                h.motorFrontLeft.setPower((float) .2);
                h.motorFrontRight.setPower((float) .2);
                h.motorBackLeft.setPower((float) .2);
                h.motorBackRight.setPower((float) .2);
                while (h.motorBackRight.getCurrentPosition() < 1020 - 20 && !isStopRequested() && h.motorFrontLeft.getCurrentPosition() > -1020 + 20
                        && h.motorFrontRight.getCurrentPosition() < 1020 - 20 && h.motorBackLeft.getCurrentPosition() > -1020 + 20) {

                }
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.sleep(1);
                h.drivePureEncoder(true, 1250, .2);
                h.sleep(1);
                h.drivePureEncoder(false, 1250, .2);
                h.drivePureEncoder(false, 1840, .2);
                h.motorLift.setTargetPosition(-1350);
                sleep(500);
                h.servoArm.setPosition(.45);
                h.sleep(500);
                h.servoClaw.setPosition(.43);
                h.sleep(500);
                h.drivePureEncoder(true, 100, .2);
                h.sleep(500);
                while(h.moveArm(0.21,-1300, 1, 0.24)){}
                sleep(1750);
                while(!h.liftLimit.isPressed()){
                    h.motorLift.setTargetPosition(-30);
                    h.motorLift.setPower(.15);
                }
                h.motorLift.setPower(0);
                break;
        }
    }
}
