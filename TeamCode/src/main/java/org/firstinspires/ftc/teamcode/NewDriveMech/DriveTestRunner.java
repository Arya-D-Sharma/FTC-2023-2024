package org.firstinspires.ftc.teamcode.NewDriveMech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.FinalArm;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.MecanumDriveHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.IntakeHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.EndgameHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.ArmHandler;

@TeleOp(name = "Fresh beard")
public class DriveTestRunner extends LinearOpMode {

    // Drive Vars
    boolean driveAllowed = true;

    double xMove;
    double yMove;
    double rX ;
    boolean lastRightState = false;
    boolean lastLeftState = false;

    int targetPosition = 0;
    boolean isFoldingOut = false;
    boolean isFoldingIn = false;

    boolean lastY;
    double multiplier = 1;

    int holdvar = 0;

    ElapsedTime tm1;

    public void runOpMode() {

        waitForStart();

        AltDriveClass drive = new AltDriveClass(hardwareMap);
        telemetry.addLine("Mecanum Handler Good");
        IntakeHandler intake = new IntakeHandler(hardwareMap);
        telemetry.addLine("Intake Handler Good");
        EndgameHandler end = new EndgameHandler(hardwareMap);
        telemetry.addLine("Endgame Handler Good");
        FinalArm outtake = new FinalArm(hardwareMap);
        telemetry.addLine("Arm Handler Good");
        telemetry.update();

        tm1 = new ElapsedTime();

        while(opModeIsActive()) {
            // Intake Runner
            if (gamepad1.right_trigger > 0.15) {
                intake.backward(gamepad1.right_trigger);

                if (outtake.arm.getCurrentPosition() < 50) {
                    outtake.d1 = true;
                    outtake.d2 = true;
                    outtake.dropUpdate();
                }

            }

            else if (gamepad1.left_trigger > 0.15) {
                intake.forward(gamepad1.left_trigger);
            }

            else {
                intake.stop();
            }

            // Drive Runner
            /*
            xMove = gamepad1.left_stick_x;
            yMove = -gamepad1.left_stick_y;
            rX = gamepad1.right_stick_x;
            */

            if (driveAllowed) {
                drive.run(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            // Endgame Methods

            if (gamepad2.start) {
                end.activate();
            }

            else if (gamepad2.back) {
                end.deactive();
            }

            if (end.active) {
                if (gamepad2.dpad_up) {
                    end.winchUp();
                } else if (gamepad2.dpad_down) {
                    end.winchDown();
                } else if (gamepad2.dpad_left) {
                    holdvar = end.winch.getCurrentPosition();
                }
                else {
                    end.winchHold(holdvar);
                }
                if (gamepad2.left_bumper) {
                    end.launch();
                }
                if (gamepad2.right_bumper) {
                    outtake.setArm(200, outtake.armPow);
                    end.extendRod();
                }
            }

            if (gamepad1.left_bumper && !lastLeftState) {
                outtake.d1 = !outtake.d1;
                outtake.dropUpdate();
            }

            if (gamepad1.right_bumper && !lastRightState) {
                outtake.d2 = !outtake.d2;
                outtake.dropUpdate();
            }

            if (gamepad1.y && !lastY) {
                if (multiplier == 1) {
                    multiplier = 0.5;
                }

                else {
                    multiplier = 1;
                }
            }

            if (gamepad2.a) {
                isFoldingOut = true;
                outtake.targetPosition = 1;

                outtake.d1 = false;
                outtake.d2 = false;
                outtake.dropUpdate();

                tm1.reset();
            }

            if (gamepad2.b) {
                isFoldingOut = true;
                outtake.targetPosition = 2;

                outtake.d1 = false;
                outtake.d2 = false;
                outtake.dropUpdate();

                tm1.reset();
            }

            if (gamepad2.y) {
                isFoldingOut = true;
                outtake.targetPosition = 3;

                outtake.d1 = false;
                outtake.d2 = false;
                outtake.dropUpdate();

                tm1.reset();
            }

            if (gamepad2.x && gamepad2.right_bumper) {
                isFoldingIn = true;
                isFoldingOut = false;
                outtake.wristIn();
                tm1.reset();
            }

            if (isFoldingOut) {

                outtake.setArm(outtake.armPos[outtake.targetPosition], outtake.armPow);

                if (Math.abs(outtake.arm.getCurrentPosition() - outtake.armPos[outtake.targetPosition]) < 30) {
                    outtake.wristOut();
                    isFoldingOut = false;
                }
            }

            if (isFoldingIn) {

                if (tm1.milliseconds() > 500) {
                    outtake.setArm(0, outtake.slowPow);
                    isFoldingIn = false;
                }
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.05 && !isFoldingOut) {
                outtake.setArm(outtake.arm.getCurrentPosition() - (int) (20*gamepad2.left_stick_y), outtake.armPow);
            }

            lastRightState = gamepad1.right_bumper;
            lastLeftState = gamepad1.left_bumper;
            lastY = gamepad1.y;
        }
    }
}