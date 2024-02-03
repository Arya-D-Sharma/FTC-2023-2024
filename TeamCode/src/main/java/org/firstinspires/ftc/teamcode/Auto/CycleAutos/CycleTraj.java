package org.firstinspires.ftc.teamcode.Auto.CycleAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.MecanumDriveHandler;
import org.firstinspires.ftc.teamcode.Vision.ColorGetter;
import org.firstinspires.ftc.teamcode.Vision.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous(name = "Cycle Traj")
public class CycleTraj extends LinearOpMode {

    ElapsedTime tm1;
    Location loc;

    @Override
    public void runOpMode() {

        tm1 = new ElapsedTime();

        FinalArm outtake = new FinalArm(hardwareMap);
        IntakeHandler intake = new IntakeHandler(hardwareMap);
        MecanumDriveHandler driveHandler = new MecanumDriveHandler(hardwareMap);
        ColorRangeSensor Csensor1;
        ColorRangeSensor Csensor2;

        Csensor1 = hardwareMap.get(ColorRangeSensor.class, "c1");
        Csensor2 = hardwareMap.get(ColorRangeSensor.class, "c2");

        Csensor1.enableLed(false);
        Csensor2.enableLed(false);

        boolean pixelOneIn = false;
        boolean pixelTwoIn = false;

        tm1.startTime();
        tm1.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(38, 9.5, Math.PI));

        Trajectory toFirst = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-55, 9.5, Math.PI))
                .build();

        Trajectory toKnown = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(84)
                .build();

        while (!isStarted() && !isStopRequested()) {
            outtake.d1 = true;
            outtake.d2 = true;
            outtake.dropUpdate();
        }

        drive.followTrajectory(toFirst);
        intake.forward(0.65);
        driveHandler.forw(0.2);

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 750) {
            time = tm1.milliseconds();
        }

        driveHandler.stop();

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 1000) {
            intake.stop();
            time = tm1.milliseconds();
        }

        intake.backward(1);
        driveHandler.backw(0.2);

        if (!(Csensor1.getDistance(DistanceUnit.CM) < 2 && Csensor2.getDistance(DistanceUnit.CM) < 2)) {
            tm1.reset();
            time = tm1.milliseconds();

            while (time < 1000) {
                time = tm1.milliseconds();
            }

            driveHandler.stop();
        }

        if (!(Csensor1.getDistance(DistanceUnit.CM) < 2 && Csensor2.getDistance(DistanceUnit.CM) < 2)) {
            tm1.reset();
            time = tm1.milliseconds();

            driveHandler.forw(0.2);

            while (time < 1000) {
                time = tm1.milliseconds();
            }

            driveHandler.stop();
        }

        if (!(Csensor1.getDistance(DistanceUnit.CM) < 2 && Csensor2.getDistance(DistanceUnit.CM) < 2)) {
            tm1.reset();
            time = tm1.milliseconds();

            driveHandler.backw(0.2);

            while (time < 1000) {
                time = tm1.milliseconds();
            }

            driveHandler.stop();
        }

        /*
        while (time < 6000 && (pixelOneIn == false || pixelTwoIn == false)) {
            time = tm1.milliseconds();
            intake.backward(1);

            if (time > 1000) {
                driveHandler.stop();
            }

            if (Csensor1.getDistance(DistanceUnit.CM) < 2) {
                pixelOneIn = true;
            } else {
                pixelOneIn = false;
            }

            if (Csensor2.getDistance(DistanceUnit.CM) < 2) {
                pixelTwoIn = true;
            } else {
                pixelTwoIn = false;
            }

            time = tm1.milliseconds();
        }

         */
        outtake.d1 = false;
        outtake.d2 = false;
        outtake.dropUpdate();
        intake.stop();

        tm1.reset();
        time = tm1.milliseconds();

        while (time < 500) {
            intake.forward(1);
            time = tm1.milliseconds();
        }

        drive.followTrajectory(toKnown);
    }
}
