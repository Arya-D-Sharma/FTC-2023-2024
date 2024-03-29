package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

@Autonomous(name="Auto Red Right")
@Disabled
public class AutoRedRight extends LinearOpMode {

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
        drive.setPoseEstimate(new Pose2d(14.4, -55.5, 4.69));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(20.9, -42.1))
                .build();

        // Left Pixel Drop
        Trajectory lpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(6.5, -32.5, 5.54))
                .build();

        Trajectory Lback = drive.trajectoryBuilder(lpDrop.end())
                .lineToSplineHeading(new Pose2d(16.4, -39.5, 5.54))
                .build();

        Trajectory Lboard = drive.trajectoryBuilder(Lback.end())
                .lineToSplineHeading(new Pose2d(65.96, -30.32, 3.10))
                .build();

        // Right Pixel Drop
        Trajectory rpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(28.47, -31.66, 4.63))
                .build();

        Trajectory Rback = drive.trajectoryBuilder(rpDrop.end())
                .lineToSplineHeading(new Pose2d(29.97, -45.31, 4.65))
                .build();

        Trajectory Rboard = drive.trajectoryBuilder(Rback.end())
                .lineToSplineHeading(new Pose2d(64.42, -45.78, 3.09))
                .build();

        // Center Pixel Drop
        Trajectory cpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(17.05, -25.82, 4.72))
                .build();

        Trajectory Cback = drive.trajectoryBuilder(cpDrop.end())
                .lineToSplineHeading(new Pose2d(18.48, -41.68, 4.68))
                .build();

        Trajectory Cboard = drive.trajectoryBuilder(Cback.end())
                .lineToSplineHeading(new Pose2d(65.81, -36.96, 3.09))
                .build();

        Trajectory park = drive.trajectoryBuilder(Lboard.end())
                .lineToSplineHeading(new Pose2d(49.38, -67.59, 1.65))
                .build();

        Trajectory back = drive.trajectoryBuilder(Lboard.end())
                .forward(12)
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

        if (loc == Location.LEFT) {
            drive.followTrajectory(lpDrop);
            drive.followTrajectory(Lback);
            drive.followTrajectory(Lboard);

            park = drive.trajectoryBuilder(Lboard.end())
                    .lineToSplineHeading(new Pose2d(49.38, -67.59, 1.65))
                    .build();

            back = drive.trajectoryBuilder(Lboard.end())
                    .forward(12)
                    .build();
        }

        else if (loc == Location.RIGHT) {
            drive.followTrajectory(rpDrop);
            drive.followTrajectory(Rback);
            drive.followTrajectory(Rboard);

            park = drive.trajectoryBuilder(Rboard.end())
                    .lineToSplineHeading(new Pose2d(49.38, -67.59, 1.65))
                    .build();

            back = drive.trajectoryBuilder(Rboard.end())
                    .forward(12)
                    .build();
        }

        else {
            drive.followTrajectory(cpDrop);
            drive.followTrajectory(Cback);
            drive.followTrajectory(Cboard);

            park = drive.trajectoryBuilder(Cboard.end())
                    .lineToSplineHeading(new Pose2d(49.38, -67.59, 1.65))
                    .build();

            back = drive.trajectoryBuilder(Cboard.end())
                    .forward(12)
                    .build();
        }

        outtake.setArm(outtake.armPos[1], outtake.armPow);

        while (Math.abs(outtake.arm.getTargetPosition() - outtake.arm.getCurrentPosition()) > 10) {
            outtake.wristIn();
        }

        outtake.wristOut();

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 500) {
            time = tm1.milliseconds();
        }

        outtake.d1 = true;
        outtake.d2 = true;
        outtake.dropUpdate();

        outtake.setArm(outtake.armPos[2], outtake.armPow);

        while (Math.abs(outtake.arm.getTargetPosition() - outtake.arm.getCurrentPosition()) > 30) {
            outtake.wristOut();
        }

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 500) {
            time = tm1.milliseconds();
        }

        outtake.wristIn();

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 500) {
            time = tm1.milliseconds();
        }

        outtake.setArm(0, outtake.slowPow);

        while (Math.abs(outtake.arm.getTargetPosition() - outtake.arm.getCurrentPosition()) > 30) {
            outtake.wristIn();
        }

        drive.followTrajectory(park);

        Trajectory corner = drive.trajectoryBuilder(park.end())
                .lineToSplineHeading(new Pose2d(70.357, -68.789, 1.65))
                .build();


        drive.followTrajectory(corner);
    }
}