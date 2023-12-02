package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Color TeleOp")
public class ColorAutoTest extends LinearOpMode {

    OpenCvCamera camera;
    int detection = 2;

    int[] leftCoords = {10, 15, 10, 20}; // x1, x2, y1, y2
    int[] centerCoords = {30, 35, 10, 20}; // x1, x2, y1, y2

    @Override
    public void runOpMode() {

        int pos1X = 124;
        int pos1Y = 310;
        int pos2X = 392;
        int pos2Y = 262;
        int pos3X = 660;
        int pos3Y = 278;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};

        ColorGetter colorDetector = new ColorGetter(points, 10, 10, false, hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Test: ", colorDetector.location);
            telemetry.addData("Center: ", colorDetector.centerfinal);
            telemetry.addData("Left: ", colorDetector.leftfinal);
            telemetry.addData("Right: ", colorDetector.rightfinal);
            telemetry.addData("Lefts: ", colorDetector.lefts[2]);
            telemetry.addData("Center: ", colorDetector.centers[2]);
            telemetry.addData("Right: ", colorDetector.rights[2]);
            telemetry.update();
        }
        //telemetry.addData("Left");
    }
}
