package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Fun With Roadrunner")
public class FunWithRoadrunner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(14.4, -55.5, 4.69));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(20.9, -42.1))
                .build();

        Trajectory pixelDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(8, -32.5, 5.54))
                .build();

        Trajectory back = drive.trajectoryBuilder(pixelDrop.end())
                .lineToSplineHeading(new Pose2d(16.4, -39.5, 5.54))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(toFirst);
        drive.followTrajectory(pixelDrop);
        drive.followTrajectory(back);
    }
}
