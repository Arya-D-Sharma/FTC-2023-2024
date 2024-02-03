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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewDriveMech.FromRoadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

import java.util.Arrays;

@Autonomous(name="Experimental Red left")
public class JacksonCode extends LinearOpMode {

    ElapsedTime tm1;
    Location loc;

    Trajectory toPixel;
    Trajectory crossField;
    Trajectory toBoard;
    Trajectory prePark;

    @Override
    public void runOpMode() {

        tm1 = new ElapsedTime();

        FinalArm outtake = new FinalArm(hardwareMap);
        ColorGetter pipeline = new ColorGetter(10, 10, true, hardwareMap);

        tm1.startTime();
        tm1.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38, -61, 3*Math.PI/2));

        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(10),
                new AngularVelocityConstraint(1)
        ));


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


        if (loc == Location.LEFT) {

            toPixel = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(-38, -49),3*Math.PI/2)
                    .splineTo(new Vector2d(-46, -41),3*Math.PI/2)
                    .forward(8)
                    .build();

            crossField = drive.trajectoryBuilder(toPixel.end())
                    .splineTo(new Vector2d(-57, -45), 3*Math.PI/2)
                    .splineTo(new Vector2d(-55, -11), 3*Math.PI/2)
                    .splineTo(new Vector2d(-44, -12.5), Math.PI)
                    .back(86)
                    .strafeLeft(24)
                    .splineTo(new Vector2d(56, -33.5), Math.PI)
                    .build();

            toBoard = drive.trajectoryBuilder(crossField.end())

                    .build();

            prePark = drive.trajectoryBuilder(crossField.end())
                    .lineToSplineHeading(new Pose2d(61.5, -51, Math.PI))
                    .build();

            park = drive.trajectoryBuilder(prePark.end())
                    .lineToSplineHeading(new Pose2d(56, -52, Math.PI/2))
                    .build();
        }

        else if (loc == Location.RIGHT) {
            toPixel = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(-38, -49),3*Math.PI/2)
                    .splineTo(new Vector2d(-29, -37), 3.71)
                    .forward(8)
                    .build();

            crossField = drive.trajectoryBuilder(toPixel.end())
                    .splineTo(new Vector2d(-57, -45), 3*Math.PI/2)
                    .splineTo(new Vector2d(-55, -11), 3*Math.PI/2)
                    .splineTo(new Vector2d(-44, -12.5), Math.PI)
                    .back(86)
                    .strafeLeft(24)
                    .splineTo(new Vector2d(56, -45), Math.PI)
                    .build();


            prePark = drive.trajectoryBuilder(crossField.end())
                    .lineToSplineHeading(new Pose2d(61.5, -51, Math.PI))
                    .build();

            park = drive.trajectoryBuilder(prePark.end())
                    .lineToSplineHeading(new Pose2d(56, -52, Math.PI/2))
                    .build();

        }

        else {
            toPixel = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(-38, -49),3*Math.PI/2)
                    .splineTo(new Vector2d(-44.5, -29), 3.82)
                    .forward(8)
                    .build();

            crossField = drive.trajectoryBuilder(toPixel.end())
                    .splineTo(new Vector2d(-57, -45), 3*Math.PI/2)
                    .splineTo(new Vector2d(-55, -11), 3*Math.PI/2)
                    .splineTo(new Vector2d(-44, -12.5), Math.PI)
                    .back(86)
                    .strafeLeft(24)
                    .splineTo(new Vector2d(57, -37), Math.PI)
                    .build();

            prePark = drive.trajectoryBuilder(crossField.end())
                    .lineToSplineHeading(new Pose2d(61.5, -51, Math.PI))
                    .build();

            park = drive.trajectoryBuilder(prePark.end())
                    .lineToSplineHeading(new Pose2d(52, -56, Math.PI/2))
                    .build();

        }

        drive.followTrajectory(toPixel);
        drive.followTrajectory(crossField);

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

        PoseStorage.currentPose = drive.getPoseEstimate();
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