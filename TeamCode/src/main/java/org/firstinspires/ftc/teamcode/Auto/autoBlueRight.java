package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Auto Blue Right")
@Disabled
public class autoBlueRight extends LinearOpMode {

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
        drive.setPoseEstimate(new Pose2d(-27.298, 55.595, 1.627));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-32.709, 42.436, 1.646))
                .build();

        Trajectory Move1 = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-52.441, 37.707, 1.672))
                .build();

        Trajectory Move2 = drive.trajectoryBuilder(Move1.end())
                .lineToSplineHeading(new Pose2d(-48.744, 4.315, 1.655))
                .build();

        Trajectory rotateIn = drive.trajectoryBuilder(Move2.end())
                .lineToSplineHeading(new Pose2d(-31.968, 10.512, 3.243))
                .build();

        Trajectory Move3 = drive.trajectoryBuilder(rotateIn.end())
                .lineToSplineHeading(new Pose2d(44.388, 16.512, 3.23))
                .build();

        // Left Pixel Drop
        Trajectory lpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-19.584, 31.714, 2.356))
                .build();

        Trajectory Lback = drive.trajectoryBuilder(lpDrop.end())
                .lineToSplineHeading(new Pose2d(-33.014, 44.555, 2.480))
                .build();

        Trajectory Lboard = drive.trajectoryBuilder(Move3.end())
                .lineToSplineHeading(new Pose2d(66.874, 44.914, 3.130))
                .build();

        // Right Pixel Drop
        Trajectory rpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-41.565, 31.428, 1.725))
                .build();

        Trajectory Rback = drive.trajectoryBuilder(rpDrop.end())
                .lineToSplineHeading(new Pose2d(-43.230, 41.296, 1.717))
                .build();

        Trajectory Rboard = drive.trajectoryBuilder(Move3.end())
                .lineToSplineHeading(new Pose2d(66.393, 34.262, 3.137))
                .build();

        // Center Pixel Drop
        Trajectory cpDrop = drive.trajectoryBuilder(toFirst.end())
                .lineToSplineHeading(new Pose2d(-29.988, 27.398, 1.68))
                .build();

        Trajectory Cback = drive.trajectoryBuilder(cpDrop.end())
                .lineToSplineHeading(new Pose2d(-31.427, 34.363, 1.706))
                .build();

        Trajectory Cboard = drive.trajectoryBuilder(Move3.end())
                .lineToSplineHeading(new Pose2d(66.618, 41.262, 3.153))
                .build();

        Trajectory park = drive.trajectoryBuilder(Lboard.end())
                .lineToSplineHeading(new Pose2d(39.357, 68.789, 4.698))
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

            Move1 = drive.trajectoryBuilder(Lback.end())
                    .lineToSplineHeading(new Pose2d(-52.441, 37.707, 1.672))
                    .build();

            drive.followTrajectory(Move1);
            drive.followTrajectory(Move2);
            drive.followTrajectory(rotateIn);
            drive.followTrajectory(Move3);
            drive.followTrajectory(Lboard);

            park = drive.trajectoryBuilder(Lboard.end())
                    .lineToSplineHeading(new Pose2d(44.388, 24.512, 4.698))
                    .build();
        }

        else if (loc == Location.RIGHT) {
            drive.followTrajectory(rpDrop);
            drive.followTrajectory(Rback);

            Move1 = drive.trajectoryBuilder(Rback.end())
                    .lineToSplineHeading(new Pose2d(-52.441, 37.707, 1.672))
                    .build();

            drive.followTrajectory(Move1);
            drive.followTrajectory(Move2);
            drive.followTrajectory(rotateIn);
            drive.followTrajectory(Move3);
            drive.followTrajectory(Rboard);

            park = drive.trajectoryBuilder(Rboard.end())
                    .lineToSplineHeading(new Pose2d(44.388, 24.512, 4.698))
                    .build();
        }

        else {
            drive.followTrajectory(cpDrop);
            drive.followTrajectory(Cback);

            Move1 = drive.trajectoryBuilder(Cback.end())
                    .lineToSplineHeading(new Pose2d(-52.441, 37.707, 1.672))
                    .build();

            drive.followTrajectory(Move1);
            drive.followTrajectory(Move2);
            drive.followTrajectory(rotateIn);
            drive.followTrajectory(Move3);
            drive.followTrajectory(Cboard);

            park = drive.trajectoryBuilder(Cboard.end())
                    .lineToSplineHeading(new Pose2d(44.388, 24.512, 4.698))
                    .build();
        }

        outtake.setArm(outtake.armPos[1] - 200, outtake.armPow);

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
                .lineToSplineHeading(new Pose2d(70.388, 30, 4.777))
                .build();

        drive.followTrajectory(corner);
    }
}
