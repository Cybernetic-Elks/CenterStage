package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//TODO Make it go to a high pole
@Disabled
@Autonomous(name="BASIC CENTERSTAGE1", group="Auto")
public class CenterStageAutonomous extends LinearOpMode {
    Hardware h = new Hardware();
    IMU imu;
    OpenCvCamera webCam;

    public enum Side {
        ONE,
        TWO,
        THREE
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
        h.turnIMU(90,.2,.1);
    }
}