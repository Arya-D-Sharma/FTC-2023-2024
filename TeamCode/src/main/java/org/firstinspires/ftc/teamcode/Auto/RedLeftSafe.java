package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

@Autonomous(name="Red Left Safe")
public class RedLeftSafe extends LinearOpMode {

    ElapsedTime tm1;
    Location loc;

    @Override
    public void runOpMode() {

        tm1 = new ElapsedTime();

        FinalArm outtake = new FinalArm(hardwareMap);
        ColorGetter pipeline = new ColorGetter(10, 10, true, hardwareMap);

        tm1.startTime();
        tm1.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-23.4, -55.5, 4.69));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-23.22, -46.52, 4.69))
                .build();

        // Left Pixel Drop
        Trajectory lpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-30.2, -32.3, 4.67))
                .build();

        Trajectory Lback = drive.trajectoryBuilder(lpDrop.end())
                .lineToSplineHeading(new Pose2d(-30.5, -41.87, 4.67))
                .build();

        // End of edits 9:09

        // Right Pixel Drop
        Trajectory rpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-9.17, -31.41, 4))
                .build();

        Trajectory Rback = drive.trajectoryBuilder(rpDrop.end())
                .lineToSplineHeading(new Pose2d(-17.45, -40.59, 4))
                .build();

        // Center Pixel Drop
        Trajectory cpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-18.85, -26.17, 4.72))
                .build();

        Trajectory Cback = drive.trajectoryBuilder(cpDrop.end())
                .lineToSplineHeading(new Pose2d(-18.62, -38.23, 4.73))
                .build();

        Trajectory straighten = drive.trajectoryBuilder(Lback.end())
                .lineToSplineHeading(new Pose2d(-23.4, -55.5, 4.69-Math.PI))
                .build();

        while (!isStarted() && !isStopRequested()) {

            outtake.d1 = false;
            outtake.d2 = false;
            outtake.dropUpdate();

            if (pipeline.lefts != null && pipeline.rights != null && pipeline.centers != null) {
                loc = pipeline.location;
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

        drive.followTrajectory(toFirst);

        if (loc == Location.RIGHT) {
            drive.followTrajectory(rpDrop);
            drive.followTrajectory(Rback);

            straighten = drive.trajectoryBuilder(Rback.end())
                    .lineToSplineHeading(new Pose2d(-23.4, -55.5, 4.69-Math.PI))
                    .build();

            drive.followTrajectory(straighten);
        }

        else if (loc == Location.LEFT) {
            drive.followTrajectory(lpDrop);
            drive.followTrajectory(Lback);

            straighten = drive.trajectoryBuilder(Lback.end())
                    .lineToSplineHeading(new Pose2d(-23.4, -55.5, 4.69-Math.PI))
                    .build();

            drive.followTrajectory(straighten);
        }

        else {
            drive.followTrajectory(cpDrop);
            drive.followTrajectory(Cback);

            straighten = drive.trajectoryBuilder(Cback.end())
                    .lineToSplineHeading(new Pose2d(-23.4, -55.5, 4.69-Math.PI))
                    .build();

            drive.followTrajectory(straighten);
        }
    }
}
