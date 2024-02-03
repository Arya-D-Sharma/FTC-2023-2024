package org.firstinspires.ftc.teamcode.Auto.NeoAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewDriveMech.FromRoadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="A0 New Blue Left")
public class BlueLeftNeo extends LinearOpMode {

    ElapsedTime tm1;
    Location loc;

    @Override
    public void runOpMode() {

        tm1 = new ElapsedTime();

        FinalArm outtake = new FinalArm(hardwareMap);
        ColorGetter pipeline = new ColorGetter(10, 10, false, hardwareMap);

        tm1.startTime();
        tm1.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(18, 60.5, Math.PI/2));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(15, 51, Math.PI/2))
                .build();

        // Left Pixel Drop
        Trajectory lpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(25.5, 40, Math.PI/2))
                .build();

        Trajectory Lback = drive.trajectoryBuilder(lpDrop.end())
                .forward(12)
                .build();

        Trajectory Lboard = drive.trajectoryBuilder(Lback.end())
                .lineToSplineHeading(new Pose2d(55, 36.5, Math.PI))
                .build();

        // Right Pixel Drop
        Trajectory rpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(8.5, 39, 0.757))
                .build();

        Trajectory Rback = drive.trajectoryBuilder(rpDrop.end())
                .forward(12)
                .build();

        Trajectory Rboard = drive.trajectoryBuilder(Rback.end())
                .lineToSplineHeading(new Pose2d(55, 25.6, Math.PI))
                .build();

        // Center Pixel Drop
        Trajectory cpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(22, 29.5, 0.917))
                .build();

        Trajectory Cback = drive.trajectoryBuilder(cpDrop.end())
                .forward(12)
                .build();

        Trajectory Cboard = drive.trajectoryBuilder(Cback.end())
                .lineToSplineHeading(new Pose2d(55, 30, Math.PI))
                .build();

        Trajectory Prepark = drive.trajectoryBuilder(Lboard.end())
                .lineToSplineHeading(new Pose2d(47, 45, Math.PI))
                .build();

        Trajectory park = drive.trajectoryBuilder(Prepark.end())
                .lineToSplineHeading(new Pose2d(52, 60, 3*Math.PI/2 - Math.toRadians(5)))
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

            Prepark = drive.trajectoryBuilder(Lboard.end())
                    .lineToSplineHeading(new Pose2d(47, 45, Math.PI))
                    .build();
        }

        else if (loc == Location.RIGHT) {
            drive.followTrajectory(rpDrop);
            drive.followTrajectory(Rback);
            drive.followTrajectory(Rboard);

            Prepark = drive.trajectoryBuilder(Rboard.end())
                    .lineToSplineHeading(new Pose2d(47, 45, Math.PI))
                    .build();
        }

        else {
            drive.followTrajectory(cpDrop);
            drive.followTrajectory(Cback);
            drive.followTrajectory(Cboard);

            Prepark = drive.trajectoryBuilder(Cboard.end())
                    .lineToSplineHeading(new Pose2d(47, 45, Math.PI))
                    .build();
        }

        outtake.setArm(outtake.armPos[1]-125, outtake.armPow);

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
        drive.followTrajectory(Prepark);
        drive.followTrajectory(park);

        PoseStorage.currentPose = drive.getPoseEstimate();

        /*
        Trajectory corner = drive.trajectoryBuilder(park.end())
                .lineToSplineHeading(new Pose2d(70.357, -68.789, 1.65))
                .build();
        */

        //drive.followTrajectory(corner);
    }
}