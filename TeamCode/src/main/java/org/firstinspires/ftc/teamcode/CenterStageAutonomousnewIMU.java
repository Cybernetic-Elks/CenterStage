package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.openftc.easyopencv.OpenCvCamera;

//TODO Make it go to a high pole
//@Disabled
@Autonomous(name="BASIC CENTERSTAGE", group="Auto")
public class CenterStageAutonomousnewIMU extends LinearOpMode {
    Hardware h = new Hardware();
    IMU imu;
    //OpenCvCamera webCam;

    public enum Side {
        //ONE,
        //TWO,
        //THREE
    }

    Side side;

    @Override
    public void runOpMode() throws InterruptedException {

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().

                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SignalDetector detector = new SignalDetector(telemetry);
        webCam.setPipeline(detector);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera not able to open");
            }
        });*/
      try {
            h.init(hardwareMap, telemetry);
        } catch (
                Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        //h.motorFrontRight(DcMotorSimple.Direction.FORWARD);
        //h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        //webCam.stopStreaming();
        telemetry.addData("motorFrontLeft encoder value: ", h.motorFrontLeft.getCurrentPosition());
        telemetry.addData("motorFrontRight encoder value: ", h.motorFrontRight.getCurrentPosition());
        telemetry.addData("motorBackLeft encoder value: ", h.motorBackLeft.getCurrentPosition());
        telemetry.addData("motorBackRight encoder value: ", h.motorBackRight.getCurrentPosition());
        //telemetry.addData("Color:", detector.getSide());

        //test for now
        h.drivePureEncoder(false, h.calculateTicks(15), .2);
        h.sleep(250);
        h.drivePureEncoder(false, h.calculateTicks(15), .2);
        h.sleep(250);
        h.motorFrontLeft.setPower(.2);
        h.motorBackLeft.setPower(.2);
        h.motorFrontRight.setPower(.2);
        h.motorBackRight.setPower(.2);
        h.sleep(1500);
        h.motorFrontLeft.setPower(0);
        h.motorBackLeft.setPower(0);
        h.motorFrontRight.setPower(0);
        h.motorBackRight.setPower(0);
    }
}