package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetector extends OpenCvPipeline {
    /**
     * Programmer:    Sean Pakros
     * Date Created:  12/15/21
     * Purpose: Code for detecting our shipping element using openCV.
     * This is based off of another teams code (Wolf Corp Robotics 12525)
     * for detecting skystones in a previous season which I then modified to work for this season and our shipping element.
     */
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        MIDDLE,
        RIGHT,
        LEFT
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 140),
            new Point(130, 190));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(270, 140),
            new Point(315, 190));
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    public PropDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(170, 40, 40);
        Scalar highHSV = new Scalar(180, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat middle = mat.submat(LEFT_ROI);
        Mat right = mat.submat(MIDDLE_ROI);


        double leftValue = Core.sumElems(middle).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(right).val[0] / MIDDLE_ROI.area() / 255;

        middle.release();
        right.release();

//        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
//        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
//        telemetry.addData("Middle percentage", Math.round(leftValue * 100) + "%");
//        telemetry.addData("Right percentage", Math.round(middleValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if (!stoneLeft && !stoneMiddle) {
            location = Location.RIGHT;
            telemetry.addData("ShippingElement Location", "RIGHT");
        } else if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("ShippingElement Location", "LEFT");
        } else {
            location = Location.MIDDLE;
            telemetry.addData("ShippingElement Location", "MIDDLE");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorEmpty = new Scalar(255, 0, 0);
        Scalar colorShipping = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.MIDDLE ? colorEmpty : colorShipping);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.RIGHT ? colorEmpty : colorShipping);

        if (location == Location.LEFT) {
            Imgproc.rectangle(mat, LEFT_ROI, colorEmpty);
            Imgproc.rectangle(mat, MIDDLE_ROI, colorEmpty);
        }
        return mat;
    }

    public Location getLocation() {
        return location;
    }
}