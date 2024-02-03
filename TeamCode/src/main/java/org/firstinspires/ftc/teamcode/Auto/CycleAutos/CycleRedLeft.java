package org.firstinspires.ftc.teamcode.Auto.CycleAutos;
import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.IntakeHandler;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

import java.util.Arrays;

@Autonomous(name="A0 Cycle Red Left")
public class CycleRedLeft extends LinearOpMode {

    ElapsedTime tm1;
    Location loc;

    @Override
    public void runOpMode() {

        tm1 = new ElapsedTime();

        FinalArm outtake = new FinalArm(hardwareMap);
        IntakeHandler intake = new IntakeHandler(hardwareMap);
        ColorGetter pipeline = new ColorGetter(10, 10, true, hardwareMap);
        ColorRangeSensor Csensor1;
        ColorRangeSensor Csensor2;
        LED l1;
        LED l2;

        Csensor1 = hardwareMap.get(ColorRangeSensor.class, "c1");
        Csensor2 = hardwareMap.get(ColorRangeSensor.class, "c2");
        l1 = hardwareMap.get(LED.class, "l1");
        l2 = hardwareMap.get(LED.class, "l2");

        Csensor1.enableLed(false);
        Csensor2.enableLed(false);

        boolean pixelOneIn = false;
        boolean pixelTwoIn = false;

        tm1.startTime();
        tm1.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38, -61, 3*Math.PI/2));

        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(10),
                new AngularVelocityConstraint(1)
        ));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-38, -49, 3*Math.PI/2))
                .build();

        // Left Pixel Drop
        Trajectory lpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-46, -41, 3*Math.PI/2))
                .build();

        Trajectory Lback = drive.trajectoryBuilder(lpDrop.end())
                .forward(8)
                .build();

        // Right Pixel Drop
        Trajectory rpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-29, -37, 3.71))
                .build();

        Trajectory Rback = drive.trajectoryBuilder(rpDrop.end())
                .forward(8)
                .build();

        // Center Pixel Drop
        Trajectory cpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-44.5, -29, 3.82))
                .build();

        Trajectory Cback = drive.trajectoryBuilder(cpDrop.end())
                .forward(8)
                .build();

        Trajectory first = drive.trajectoryBuilder(Rback.end())
                .lineToSplineHeading(new Pose2d(-57, -45, 3*Math.PI/2))
                .build();

        Trajectory second = drive.trajectoryBuilder(first.end())
                .lineToSplineHeading(new Pose2d(-55, -11, 3*Math.PI/2))
                .build();

        Trajectory third = drive.trajectoryBuilder(second.end())
                .lineToSplineHeading(new Pose2d(-44, -12.5, Math.PI))
                .build();

        Trajectory knocking = drive.trajectoryBuilder(third.end())
                .forward(13)
                .build();

        Trajectory across = drive.trajectoryBuilder(third.end())
                .back(86)
                .build();

        Trajectory traverse = drive.trajectoryBuilder(across.end())
                .strafeLeft(24)
                .build();

        Trajectory Cboard = drive.trajectoryBuilder(traverse.end())
                .lineToSplineHeading(new Pose2d(57, -37, Math.PI))
                .build();

        Trajectory Rboard = drive.trajectoryBuilder(traverse.end())
                .lineToSplineHeading(new Pose2d(56, -45, Math.PI))
                .build();

        Trajectory Lboard = drive.trajectoryBuilder(traverse.end())
                .lineToSplineHeading(new Pose2d(56, -33.5, Math.PI))
                .build();

        Trajectory Prepark = drive.trajectoryBuilder(Lboard.end())
                .lineToSplineHeading(new Pose2d(61.5, -51, Math.PI))
                .build();

        Trajectory park;

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

            first = drive.trajectoryBuilder(Lback.end())
                    .lineToSplineHeading(new Pose2d(-57, -45, 3*Math.PI/2))
                    .build();
        }

        else if (loc == Location.RIGHT) {
            drive.followTrajectory(rpDrop);
            drive.followTrajectory(Rback);

            first = drive.trajectoryBuilder(Rback.end())
                    .lineToSplineHeading(new Pose2d(-57, -45, 3*Math.PI/2))
                    .build();
        }

        else {
            drive.followTrajectory(cpDrop);
            drive.followTrajectory(Cback);


            first = drive.trajectoryBuilder(Cback.end())
                    .lineToSplineHeading(new Pose2d(-57, -45, 3*Math.PI/2))
                    .build();
        }

        drive.followTrajectory(first);
        drive.followTrajectory(second);
        drive.followTrajectory(third);
        intake.forward(0.5);
        drive.followTrajectory(knocking);

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 1000) {
            time = tm1.milliseconds();
            intake.backward(0.5);
            outtake.d1 = true;
            outtake.d2 = true;
            outtake.dropUpdate();
        }

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 6000 && (pixelOneIn == false && pixelTwoIn == false)) {
            time = tm1.milliseconds();
            intake.backward(1);

            if (Csensor1.getDistance(DistanceUnit.CM) < 2) {
                l1.enableLight(true);
                pixelOneIn = true;
                outtake.d1 = false;
                outtake.dropUpdate();
            }

            else {
                l1.enableLight(false);
                pixelOneIn = false;
                outtake.d1 = true;
                outtake.dropUpdate();
            }

            if(Csensor2.getDistance(DistanceUnit.CM) < 2) {
                l2.enableLight(true);
                pixelTwoIn = true;
                outtake.d2 = false;
                outtake.dropUpdate();
            }

            else {
                l2.enableLight(false);
                pixelTwoIn = false;
                outtake.d2 = true;
                outtake.dropUpdate();
            }
        }

        drive.followTrajectory(across);
        drive.followTrajectory(traverse);

        // CHANGE THIS
        drive.followTrajectory(Lboard);

        Prepark = drive.trajectoryBuilder(Lboard.end())
                .lineToSplineHeading(new Pose2d(61.5, -51, Math.PI))
                .build();

        park = drive.trajectoryBuilder(Prepark.end())
                .lineToSplineHeading(new Pose2d(56, -52, Math.PI/2))
                .build();

        outtake.setArm(outtake.armPos[1] + 50, outtake.armPow);

        while (Math.abs(outtake.arm.getTargetPosition() - outtake.arm.getCurrentPosition()) > 10) {
            outtake.wristIn();
        }

        outtake.wristOut();

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 1000) {
            time = tm1.milliseconds();
        }

        outtake.d1 = true;
        outtake.d2 = true;
        outtake.dropUpdate();

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 500) {
            time = tm1.milliseconds();
        }

        outtake.setArm(outtake.armPos[3], outtake.armPow);

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

        //drive.followTrajectory(Prepark);
        //drive.followTrajectory(park);

        /*
        Trajectory corner = drive.trajectoryBuilder(park.end())
                .lineToSplineHeading(new Pose2d(70.357, -68.789, 1.65))
                .build();
        */

        //drive.followTrajectory(corner);
    }
}