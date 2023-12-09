package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.opencv.core.Point;

@Autonomous(name="Color tester")
public class ColorAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Point pt0 = new Point(-100, 0);
        Point pt1 = new Point(0, 0);
        Point pt2 = new Point(100, 0);

        Point[] points = {pt0, pt1, pt2};

        ColorGetter pipeline = new ColorGetter(10, 10, true, hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Location", pipeline.location);
            telemetry.update();
        }
    }
}
