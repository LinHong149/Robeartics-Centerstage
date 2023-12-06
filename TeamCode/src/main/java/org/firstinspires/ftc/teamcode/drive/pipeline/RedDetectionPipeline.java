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

public class RedDetectionPipeline extends OpenCvPipeline {
    Scalar lowerRedLowerBound = new Scalar(0, 100, 100);
    Scalar lowerRedUpperBound = new Scalar(10, 255, 255);
    Scalar upperRedLowerBound = new Scalar(160, 100, 100);
    Scalar upperRedUpperBound = new Scalar(180, 255, 255);

    Mat mask = new Mat();
    Mat maskLowerRed = new Mat();
    Mat maskUpperRed = new Mat();
    Mat hierarchy = new Mat();

    public enum Position {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    private Position position = Position.UNKNOWN;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        // Apply a threshold for the lower red range
        Core.inRange(mask, lowerRedLowerBound, lowerRedUpperBound, maskLowerRed);

        // Apply a threshold for the upper red range
        Core.inRange(mask, upperRedLowerBound, upperRedUpperBound, maskUpperRed);

        // Combine both red masks
        Core.bitwise_or(maskLowerRed, maskUpperRed, mask);

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
