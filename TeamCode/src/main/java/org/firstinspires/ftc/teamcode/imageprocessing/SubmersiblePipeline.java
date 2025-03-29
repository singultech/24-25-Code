package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.*;
import java.util.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class RectangleCorners {
    public Point topLeft, topRight, bottomRight, bottomLeft;

    public RectangleCorners(Point[] points) {
        if (points.length == 4) {
            this.topLeft = points[0];
            this.topRight = points[1];
            this.bottomRight = points[2];
            this.bottomLeft = points[3];
        }
    }

    @Override
    public String toString() {
        return "TL: " + topLeft + ", TR: " + topRight +
                ", BR: " + bottomRight + ", BL: " + bottomLeft;
    }
}

public class SubmersiblePipeline extends OpenCvPipeline {

    private final ArrayList<RectangleCorners> detectedRectangles = new ArrayList<>();
    private final Mat rgbaBinaryMat = new Mat();
    private final Mat hierarchy = new Mat();
    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();
    private final MatOfPoint2f contour2f = new MatOfPoint2f();

    public Scalar lowerRGBA = new Scalar(176.0, 0.0, 0.0, 0.0);
    public Scalar upperRGBA = new Scalar(255.0, 255.0, 255.0, 255.0);
    public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness = 3;
    public int minArea = 10;
    public int maxArea = 1000;

    @Override
    public Mat processFrame(Mat input) {
        // Thresholding
        Core.inRange(input, lowerRGBA, upperRGBA, rgbaBinaryMat);

        // Find contours
        contours.clear();
        Imgproc.findContours(rgbaBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter by area
        contoursByArea.clear();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= minArea && area <= maxArea) {
                contoursByArea.add(contour);
            }
        }

        // Process filtered contours
        detectedRectangles.clear();
        for (MatOfPoint contour : contoursByArea) {
            contour.convertTo(contour2f, CvType.CV_32F);
            RotatedRect rect = Imgproc.minAreaRect(contour2f);

            // Extract and store rectangle corners
            Point[] rectPoints = new Point[4];
            rect.points(rectPoints);
            detectedRectangles.add(new RectangleCorners(rectPoints));

            // Draw rectangle
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], lineColor, lineThickness);
            }
        }

        return input;
    }

    public ArrayList<RectangleCorners> getDetectedRectangles() {
        return detectedRectangles;
    }
}
