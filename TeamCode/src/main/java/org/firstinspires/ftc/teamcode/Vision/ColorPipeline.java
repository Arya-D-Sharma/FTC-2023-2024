package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorPipeline extends OpenCvPipeline {

    private Mat frame;
    private int[] leftSquare;
    private int[] centerSquare;
    private int threshold;

    public ColorPipeline(int[] lCoords, int[] cCoords, int t) {
        leftSquare = lCoords;
        centerSquare = cCoords;
        threshold = t;
    }

    public Mat processFrame(Mat input) {
        frame = input;
        return input;
    }

    public double getC(int r, int c, int cIndex) {

        double[] colors = frame.get(r, c);
        return colors[cIndex];
    }

    public Mat processed() {
        return frame;
    }

    public String test() {
        return "A method is running ....";
    }

    public boolean checkLeft() {
        double squareAvg = 0;
        int counter = 0;

        for (int i = leftSquare[0]; i < leftSquare[1]; i++) {
            for (int j = leftSquare[2]; j < leftSquare[3]; j++) {
                squareAvg += frame.get(i, j)[0];
                counter += 1;
            }
        }

        squareAvg /= counter;

        return squareAvg > threshold;
    }

    public boolean checkCenter() {
        double squareAvg = 0;
        int counter = 0;

        for (int i = centerSquare[0]; i < centerSquare[1]; i++) {
            for (int j = centerSquare[2]; j < centerSquare[3]; j++) {
                squareAvg += frame.get(i, j)[0];
                counter += 1;
            }
        }

        squareAvg /= counter;

        return squareAvg > threshold;
    }
}
