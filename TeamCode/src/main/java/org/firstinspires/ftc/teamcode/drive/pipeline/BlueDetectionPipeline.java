package org.firstinspires.ftc.teamcode.drive.pipeline;


import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.MatOfPoint;

//
public class BlueDetectionPipeline extends OpenCvPipeline {
    Scalar midBlueLowerBound = new Scalar(90, 100, 70);  // Lower HSV values for mid blues
    Scalar midBlueUpperBound = new Scalar(120, 255, 255); // Upper HSV values for mid blues
    Scalar deepBlueLowerBound = new Scalar(120, 100, 70); // Lower HSV values for deeper blues
    Scalar deepBlueUpperBound = new Scalar(150, 255, 255); // Upper HSV values for deeper blues

    Mat mask = new Mat();
    Mat maskLowerBlue = new Mat();
    Mat maskUpperBlue = new Mat();
    Mat hierarchy = new Mat();

    public enum Position {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    private Position position = Position.UNKNOWN;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        // Apply a threshold for the lower red range
        Core.inRange(mask, midBlueLowerBound, midBlueUpperBound, maskLowerBlue);


        // Apply a threshold for the upper red range
        Core.inRange(mask, deepBlueLowerBound, deepBlueUpperBound, maskUpperBlue);

        // Combine both red masks
        Core.bitwise_or(maskLowerBlue, maskUpperBlue, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset position to UNKNOWN for each frame
        position = Position.UNKNOWN;

        double leftArea = 0, centerArea = 0, rightArea = 0;
        int frameCenter = input.width() / 2;
        int tolerance = input.width() / 10; // 10% of frame width as tolerance for center

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            int xCenter = rect.x + (rect.width / 2);

            if (xCenter < frameCenter - tolerance) {
                leftArea += rect.area();
            } else if (xCenter > frameCenter + tolerance) {
                rightArea += rect.area();
            } else {
                centerArea += rect.area();
            }
        }

        // Determine the majority area position
        if (leftArea > rightArea && leftArea > centerArea) {
            position = Position.LEFT;
        } else if (rightArea > leftArea && rightArea > centerArea) {
            position = Position.RIGHT;
        } else if (centerArea > leftArea && centerArea > rightArea) {
            position = Position.CENTER;
        }

        return mask; // Return the processed frame
    }


    public Position getPosition() {
        return position;
    }
}




// doesn't work. leftAvgFin = leftAvg.val[2]?
// https://www.youtube.com/watch?v=547ZUZiYfQE
//public class BlueDetectionPipeline extends OpenCvPipeline{
//    Mat YCbCr = new Mat();
//    Mat leftCrop, middleCrop, rightCrop;
//    double leftAvgFin, middleAvgFin, rightAvgFin;
//    Mat output = new Mat();
//    Scalar rectColour = new Scalar(0.0, 0.0, 255.0);
//
//    public enum Position {
//        LEFT, MIDDLE, RIGHT
//    }
//    private Position position = null;
//
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//
//        Rect leftRect = new Rect(1, 1, 213, 479);
//        Rect middleRect = new Rect(214, 1, 213, 479);
//        Rect rightRect = new Rect(427, 1, 213, 479);
//
//        input.copyTo(output);
//        Imgproc.rectangle(output, leftRect, rectColour, 2);
//        Imgproc.rectangle(output, middleRect, rectColour, 2);
//        Imgproc.rectangle(output, rightRect, rectColour, 2);
//
//        leftCrop = YCbCr.submat(leftRect);
//        middleCrop = YCbCr.submat(middleRect);
//        rightCrop = YCbCr.submat(rightRect);
//
//        Core.extractChannel(leftCrop, leftCrop, 0);
//        Core.extractChannel(middleCrop, middleCrop, 0);
//        Core.extractChannel(rightCrop, rightCrop, 0);
//
//        Scalar leftAvg = Core.mean(leftCrop);
//        Scalar middleAvg = Core.mean(middleCrop);
//        Scalar rightAvg = Core.mean(rightCrop);
//
//        leftAvgFin = leftAvg.val[2];
//        middleAvgFin = middleAvg.val[2];
//        rightAvgFin = rightAvg.val[2];
//
//        if (leftAvgFin > rightAvgFin && leftAvgFin > middleAvgFin) {
//            position = Position.LEFT;
//        } else if (rightAvgFin > leftAvgFin && rightAvgFin > middleAvgFin) {
//            position = Position.RIGHT;
//        } else {
//            position = Position.MIDDLE;
//        }
//
//        return(output);
//    }
//
//    public Position getPosition() {
//        return position;
//    }
//    public double getleftAvgFin() {
//        return leftAvgFin;
//    }
//    public double getrightAvgFin() {
//        return rightAvgFin;
//    }
//    public double getmiddleAvgFin() {
//        return middleAvgFin;
//    }
//}

