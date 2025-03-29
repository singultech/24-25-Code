package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.*;
import java.util.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    public Scalar lowerRGBA = new Scalar(176.0, 0.0, 0.0, 0.0);
    public Scalar upperRGBA = new Scalar(255.0, 255.0, 255.0, 255.0);
    private Mat rgbaBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public int minArea = 10;
    public int maxArea = 1000;
    private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

    private MatOfPoint2f contoursByArea2f = new MatOfPoint2f();
    private ArrayList<RotatedRect> contoursByAreaRotRects = new ArrayList<>();

    public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness = 3;

    private Mat inputRotRects = new Mat();

    private MatOfPoint biggestContour = null;

    @Override
    public Mat processFrame(Mat input) {
        Core.inRange(input, lowerRGBA, upperRGBA, rgbaBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(rgbaBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursByArea.clear();
        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if((area >= minArea) && (area <= maxArea)) {
                contoursByArea.add(contour);
            }
        }

        contoursByAreaRotRects.clear();
        for(MatOfPoint points : contoursByArea) {
            contoursByArea2f.release();
            points.convertTo(contoursByArea2f, CvType.CV_32F);

            contoursByAreaRotRects.add(Imgproc.minAreaRect(contoursByArea2f));
        }

        input.copyTo(inputRotRects);
        for(RotatedRect rect : contoursByAreaRotRects) {
            if(rect != null) {
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

                Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);
            }
        }

        this.biggestContour = null;
        for(MatOfPoint contour : contoursByArea) {
            if((biggestContour == null) || (Imgproc.contourArea(contour) > Imgproc.contourArea(biggestContour))) {
                this.biggestContour = contour;
            }
        }

        if(biggestContour != null) {
            Rect boundingRect = Imgproc.boundingRect(biggestContour);

            double centroidX = (boundingRect.tl().x + boundingRect.br().x) / 2;
            double centroidY = (boundingRect.tl().y + boundingRect.br().y) / 2;

            Point centroid = new Point(centroidX, centroidY);
            double contourArea = Imgproc.contourArea(biggestContour);

            Scalar crosshairCol = new Scalar(0.0, 255.0, 0.0);

            Imgproc.line(inputRotRects, new Point(centroidX - 10, centroidY), new Point(centroidX + 10, centroidY), crosshairCol, 5);
            Imgproc.line(inputRotRects, new Point(centroidX, centroidY - 10), new Point(centroidX, centroidY + 10), crosshairCol, 5);
        }

        return inputRotRects;
    }
}
