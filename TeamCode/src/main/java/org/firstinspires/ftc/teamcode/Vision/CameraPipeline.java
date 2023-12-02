package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraPipeline extends OpenCvPipeline {

    private Mat frame;
    private int[] sideSquare;
    private int[] centerSquare;
    private int threshold;

    private int centerX = 400;
    private int centerY = 224;

    public int loc = 2;

    public CameraPipeline(int[] sSquare, int[] cSquare, int t) {
        sideSquare = sSquare;
        centerSquare = cSquare;
        threshold = t;
    }

    public Mat processFrame(Mat input) {
        frame = input;
        return input;
    }

    public double getC (double r, double g, double b) {

        double k = 1 - Math.max(Math.max(r/255.0, g/255.0), b/255.0);
        double c = (1 - r/255.0 - k)/(1 - k);
        return c;
    }

    public double CheckCCenter() {
        double[] BGR = frame.get(centerX, centerY);
        return getC(BGR[0], BGR[1], BGR[2]);
    }

    public Mat test() {
        return frame;
    }
}
