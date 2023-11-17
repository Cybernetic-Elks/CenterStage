package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCamera;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
@Autonomous
public class OpenCVTest extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {

        WebcamName intakeWebcam = hardwareMap.get(WebcamName.class,"intakeWebcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewid", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(intakeWebcam, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280,720,OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {


    }

    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat centerCrop;
        double leftavgfin;
        double rightavgfin;
        double centeravgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);


        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline runnung");

            Rect leftRect = new Rect(1, 1, 300, 720);
            Rect rightRect = new Rect(980, 1, 300, 720);
            Rect centerRect = new Rect(300, 1, 680, 720);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            centerCrop = YCbCr.submat(centerRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(centerCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar centeravg = Core.mean(centerCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            centeravgfin = centeravg.val[0];

            if (leftavgfin > rightavgfin && leftavgfin > centeravgfin){
                telemetry.addLine("Left");
            }
            else if (leftavgfin < rightavgfin && rightavgfin > centeravgfin){
                telemetry.addLine("right");
            }
            else if (centeravgfin > rightavgfin && leftavgfin < centeravgfin) {
                telemetry.addLine("center");
            }
            return(outPut);
        }
    }
}