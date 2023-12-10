package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.opencv.core.Point;

@Autonomous(name="Color tester")
public class ColorAuto extends LinearOpMode {

    ElapsedTime tm1;

    @Override
    public void runOpMode() {
        Point pt0 = new Point(-100, 0);
        Point pt1 = new Point(-20, -75);
        Point pt2 = new Point(100, 0);

        tm1 = new ElapsedTime();

        Point[] points = {pt0, pt1, pt2};

        ColorGetter pipeline = new ColorGetter(10, 10, true, hardwareMap);

        tm1.startTime();
        tm1.reset();

        while (!isStarted() && !isStopRequested()) {

            if (pipeline.lefts != null && pipeline.rights != null && pipeline.centers != null) {
                telemetry.addData("Location", pipeline.location);
                telemetry.addData("Left R", 2 * pipeline.lefts[0] - pipeline.lefts[(0 + 1) % 3] - pipeline.lefts
                        [(0 + 2) % 3]);
                telemetry.addData("Right R", 2 * pipeline.rights[0] - pipeline.rights[(0 + 1) % 3] - pipeline.rights
                        [(0 + 2) % 3]);
                telemetry.addData("Center R", 2 * pipeline.centers[0] - pipeline.centers[(0 + 1) % 3] - pipeline.centers
                        [(0 + 2) % 3]);

                telemetry.update();
            }

        }

        while(opModeIsActive()) {

        }
    }
}
