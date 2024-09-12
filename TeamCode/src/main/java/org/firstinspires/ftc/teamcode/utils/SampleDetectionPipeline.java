package org.firstinspires.ftc.teamcode.utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline {

    // Threshold values for detecting colors
    Scalar lowerRed = new Scalar(125, 0, 0);
    Scalar upperRed = new Scalar(255, 100, 100);

    // Minimum size for detected objects
    double minSide = 50.0;

    // Store located items
    List<DetectedItem> locatedItems = new ArrayList<>();
    int id = 0;

    public Mat processImage(Mat inputImage) {
        Mat hsvImage = new Mat();
        Mat mask = new Mat();
        Mat binaryMask = new Mat();
        Mat grayImage = new Mat();

        // Convert image to HSV (if required)
        Imgproc.cvtColor(inputImage, hsvImage, Imgproc.COLOR_RGB2BGR);

        // Threshold the image for red color
        Core.inRange(hsvImage, lowerRed, upperRed, mask);

        // Convert the mask to grayscale
        Imgproc.cvtColor(mask, grayImage, Imgproc.COLOR_GRAY2BGR);

        // Apply binary threshold
        Imgproc.threshold(mask, binaryMask, 200, 255, Imgproc.THRESH_BINARY);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binaryMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            // Get the width, height, and angle of the rectangle
            double width = rotatedRect.size.width;
            double height = rotatedRect.size.height;

            // Check if the object is big enough
            if (width < minSide || height < minSide) continue;

            // Get the angle of the longest side
            Point[] boxPoints = new Point[4];
            rotatedRect.points(boxPoints);

            double[] sideLengths = {
                    distance(boxPoints[0], boxPoints[1]),
                    distance(boxPoints[1], boxPoints[2]),
                    distance(boxPoints[2], boxPoints[3]),
                    distance(boxPoints[3], boxPoints[0])
            };

            int longestSideIndex = getLongestSideIndex(sideLengths);
            Point p1 = boxPoints[longestSideIndex];
            Point p2 = boxPoints[(longestSideIndex + 1) % 4];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double angleLongestSide = Math.atan2(dy, dx) * 180.0 / Math.PI;

            double angleAdjusted = 90 - angleLongestSide;
            if (angleAdjusted > 90) {
                angleAdjusted -= 180;
            } else if (angleAdjusted < -90) {
                angleAdjusted += 180;
            }

            // Draw rectangle and add text to the image
            for (int j = 0; j < 4; j++) {
                Imgproc.line(grayImage, boxPoints[j], boxPoints[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            Point center = new Point(rotatedRect.center.x, rotatedRect.center.y);
            Imgproc.circle(grayImage, center, 5, new Scalar(0, 255, 0), -1);
            Imgproc.putText(grayImage, String.format("%.2f #%d", angleAdjusted, id),
                    new Point(center.x - 25, center.y + 25),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);

            // Store the detected item
            locatedItems.add(new DetectedItem(id, angleAdjusted, center.x, center.y, boxPoints));
            id++;
        }

        // Validate detected items and annotate invalid ones
        validateDetectedItems(grayImage);

        // Save the processed image
        Imgcodecs.imwrite("output.png", grayImage);

        return grayImage;
    }

    // Helper method to calculate distance between two points
    private double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    // Helper method to get index of the longest side
    private int getLongestSideIndex(double[] sideLengths) {
        int index = 0;
        for (int i = 1; i < sideLengths.length; i++) {
            if (sideLengths[i] > sideLengths[index]) {
                index = i;
            }
        }
        return index;
    }

    // Validate located items based on distance between corners
    private void validateDetectedItems(Mat image) {
        for (int i = 0; i < locatedItems.size(); i++) {
            DetectedItem item = locatedItems.get(i);
            for (DetectedItem otherItem : locatedItems) {
                if (item.id != otherItem.id) {
                    if (checkDistance(item.cornerPoints, otherItem.cornerPoints)) {
                        item.isValid = false;
                        Imgproc.putText(image, "invalid", item.getCenter(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                    } else {
                        item.isValid = true;
                    }
                }
            }
        }
    }

    // Check the perpendicular distance between corners (stub function, you will need to implement the logic)
    private boolean checkDistance(Point[] itemCorners, Point[] otherCorners) {
        // Convert OpenCV Points to your SampleDist Point
        SampleDist.Point[] itemCornersConverted = new SampleDist.Point[itemCorners.length];
        SampleDist.Point[] otherCornersConverted = new SampleDist.Point[otherCorners.length];

        for (int i = 0; i < itemCorners.length; i++) {
            itemCornersConverted[i] = new SampleDist.Point(itemCorners[i].x, itemCorners[i].y);
        }

        for (int i = 0; i < otherCorners.length; i++) {
            otherCornersConverted[i] = new SampleDist.Point(otherCorners[i].x, otherCorners[i].y);
        }

        // Define a threshold for minimum distance (adjust based on your needs)
        double threshold = 50.0;

        // Use SampleDist's isValid function to determine if the distance is valid
        return SampleDist.isValid(itemCornersConverted, otherCornersConverted, threshold);
    }


    // Detected item class to store properties of each detected object
    private static class DetectedItem {
        int id;
        double angle;
        double centerX;
        double centerY;
        Point[] cornerPoints;
        boolean isValid = true;

        DetectedItem(int id, double angle, double centerX, double centerY, Point[] cornerPoints) {
            this.id = id;
            this.angle = angle;
            this.centerX = centerX;
            this.centerY = centerY;
            this.cornerPoints = cornerPoints;
        }

        public Point getCenter() {
            return new Point(centerX, centerY);
        }
    }
}
