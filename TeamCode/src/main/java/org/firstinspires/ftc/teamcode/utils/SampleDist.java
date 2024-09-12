package org.firstinspires.ftc.teamcode.utils;

public class SampleDist {
    public static class Point {
        public double x;
        public double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    public static double[] lineEquation(Point p1, Point p2) {
        double A = p2.y - p1.y;
        double B = p1.x - p2.x;
        double C = A * p1.x + B * p1.y;
        return new double[]{A, B, -C};
    }

    public static double pointLineDistance(Point p, double[] line) {
        double A = line[0];
        double B = line[1];
        double C = line[2];
        return Math.abs(A * p.x + B * p.y + C) / Math.sqrt(A * A + B * B);
    }

    public static Point pointProjectionOnLine(Point p, double[] line) {
        double A = line[0];
        double B = line[1];
        double C = line[2];
        double x0 = p.x;
        double y0 = p.y;

        if (A == 0 && B == 0) {
            return null;
        }

        double denominator = A * A + B * B;
        double x = (B * (B * x0 - A * y0) - A * C) / denominator;
        double y = (A * (-B * x0 + A * y0) - B * C) / denominator;

        return new Point(x, y);
    }

    public static boolean pointWithinSegment(Point p, Point segStart, Point segEnd) {
        double x = p.x;
        double y = p.y;
        double x1 = segStart.x;
        double y1 = segStart.y;
        double x2 = segEnd.x;
        double y2 = segEnd.y;

        double dotProduct = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1);
        double segLengthSq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        double t = dotProduct / segLengthSq;

        return t >= 0 && t <= 1;
    }

    public static Point[] rectangleLongSides(Point[] rect) {
        double d1 = distance(rect[0], rect[1]);
        double d2 = distance(rect[1], rect[2]);

        if (d1 >= d2) {
            return new Point[]{rect[0], rect[1], rect[2], rect[3]};
        } else {
            return new Point[]{rect[1], rect[2], rect[3], rect[0]};
        }
    }

    public static boolean isValid(Point[] rect1, Point[] rect2, double threshold) {
        Point[] longSides1 = rectangleLongSides(rect1);
        Point[] longSides2 = rectangleLongSides(rect2);

        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < longSides1.length - 1; i++) {
            double[] line1 = lineEquation(longSides1[i], longSides1[i + 1]);

            for (int j = 0; j < longSides2.length - 1; j++) {
                double[] line2 = lineEquation(longSides2[j], longSides2[j + 1]);

                for (Point p1 : longSides1) {
                    Point projection = pointProjectionOnLine(p1, line2);

                    if (projection != null && pointWithinSegment(projection, longSides2[j], longSides2[j + 1])) {
                        double distance = pointLineDistance(p1, line2);
                        minDistance = Math.min(minDistance, distance);
                    }
                }

                for (Point p2 : longSides2) {
                    Point projection = pointProjectionOnLine(p2, line1);

                    if (projection != null && pointWithinSegment(projection, longSides1[i], longSides1[i + 1])) {
                        double distance = pointLineDistance(p2, line1);
                        minDistance = Math.min(minDistance, distance);
                    }
                }
            }
        }

        return minDistance < threshold;
    }

    private static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }

}