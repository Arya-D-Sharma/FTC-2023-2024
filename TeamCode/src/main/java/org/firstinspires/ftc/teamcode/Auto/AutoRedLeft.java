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

@Autonomous(name="Auto Red Left")
@Disabled
public class AutoRedLeft extends LinearOpMode {

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

        Trajectory Move1 = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-42, -49, 4.69))
                .build();

        Trajectory Move2 = drive.trajectoryBuilder(Move1.end())
                .lineToSplineHeading(new Pose2d(-38, -4, 4.69))
                .build();

        Trajectory rotateIn = drive.trajectoryBuilder(Move2.end())
                .lineToSplineHeading(new Pose2d(-23, -11, 3.10))
                .build();

        Trajectory Move3 = drive.trajectoryBuilder(rotateIn.end())
                .lineToSplineHeading(new Pose2d(45, -11, 3.1))
                .build();

        // Left Pixel Drop
        Trajectory lpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-30.2, -32.3, 4.67))
                .build();

        Trajectory Lback = drive.trajectoryBuilder(lpDrop.end())
                .lineToSplineHeading(new Pose2d(-30.5, -41.87, 4.67))
                .build();

        Trajectory Lboard = drive.trajectoryBuilder(Move3.end())
                .lineToSplineHeading(new Pose2d(79.15, -16.9, 3.10))
                .build();

        // End of edits 9:09

        // Right Pixel Drop
        Trajectory rpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-9.17, -31.41, 4))
                .build();

        Trajectory Rback = drive.trajectoryBuilder(rpDrop.end())
                .lineToSplineHeading(new Pose2d(-17.45, -40.59, 4))
                .build();

        Trajectory Rboard = drive.trajectoryBuilder(Move3.end())
                .lineToSplineHeading(new Pose2d(81.40, -29.58, 3.10))
                .build();

        // Center Pixel Drop
        Trajectory cpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-18.85, -26.17, 4.72))
                .build();

        Trajectory Cback = drive.trajectoryBuilder(cpDrop.end())
                .lineToSplineHeading(new Pose2d(-18.62, -38.23, 4.73))
                .build();

        Trajectory Cboard = drive.trajectoryBuilder(Move3.end())
                .lineToSplineHeading(new Pose2d(80.32, -22.3, 3.10))
                .build();

        Trajectory park = drive.trajectoryBuilder(Lboard.end())
                .lineToSplineHeading(new Pose2d(49.38, -67.59, 1.65))
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

            Move1 = drive.trajectoryBuilder(Rback.end())
                    .lineToSplineHeading(new Pose2d(-42, -49, 4.69))
                    .build();

            drive.followTrajectory(Move1);
            drive.followTrajectory(Move2);
            drive.followTrajectory(rotateIn);
            drive.followTrajectory(Move3);
            drive.followTrajectory(Rboard);

            park = drive.trajectoryBuilder(Rboard.end())
                    .lineToSplineHeading(new Pose2d(39.357, 68.789, 4.698))
                    .build();
        }

        else if (loc == Location.LEFT) {
            drive.followTrajectory(lpDrop);
            drive.followTrajectory(Lback);

            Move1 = drive.trajectoryBuilder(Lback.end())
                    .lineToSplineHeading(new Pose2d(-42, -49, 4.69))
                    .build();

            drive.followTrajectory(Move1);
            drive.followTrajectory(Move2);
            drive.followTrajectory(rotateIn);
            drive.followTrajectory(Move3);
            drive.followTrajectory(Lboard);

            park = drive.trajectoryBuilder(Lboard.end())
                    .lineToSplineHeading(new Pose2d(39.357, 68.789, 4.698))
                    .build();
        }

        else {
            drive.followTrajectory(cpDrop);
            drive.followTrajectory(Cback);

            Move1 = drive.trajectoryBuilder(Cback.end())
                    .lineToSplineHeading(new Pose2d(-42, -49, 4.69))
                    .build();

            drive.followTrajectory(Move1);
            drive.followTrajectory(Move2);
            drive.followTrajectory(rotateIn);
            drive.followTrajectory(Move3);
            drive.followTrajectory(Cboard);

            park = drive.trajectoryBuilder(Cboard.end())
                    .lineToSplineHeading(new Pose2d(39.357, 68.789, 4.698))
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

        outtake.setArm(outtake.armPos[1] + 200, outtake.armPow);

        while (Math.abs(outtake.arm.getTargetPosition() - outtake.arm.getCurrentPosition()) > 30) {
            outtake.wristOut();
        }

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 1000) {
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
                .lineToSplineHeading(new Pose2d(39.357, 68.789, 4.698))
                .build();
    }
}
