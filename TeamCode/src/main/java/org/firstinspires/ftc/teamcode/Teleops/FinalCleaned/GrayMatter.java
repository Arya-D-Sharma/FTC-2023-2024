package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.MecanumDriveHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.IntakeHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.EndgameHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.ArmHandler;

@TeleOp(name = "Greenbeard's Grey Matter")
public class GrayMatter extends LinearOpMode {

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

    int holdvar = 0;

    ElapsedTime tm1;

    public void runOpMode() {

        waitForStart();

        MecanumDriveHandler drive = new MecanumDriveHandler(hardwareMap);
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
                outtake.d1 = true;
                outtake.d2 = true;
                outtake.dropUpdate();
            }

            else if (gamepad1.left_trigger > 0.15) {
                intake.forward(gamepad1.left_trigger);
            }

            else {
                intake.stop();
            }

            // Drive Runner
            xMove = gamepad1.left_stick_x;
            yMove = -gamepad1.left_stick_y;
            rX = gamepad1.right_stick_x;

            if (driveAllowed) {
                drive.run(xMove, yMove, rX);
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
                }
                else if (gamepad2.dpad_left) {
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

            if (gamepad2.a) {
                isFoldingOut = true;
                outtake.targetPosition = 1;
                outtake.wristIn();
                tm1.reset();
            }

            if (gamepad2.b) {
                isFoldingOut = true;
                outtake.targetPosition = 2;
                outtake.wristIn();
                tm1.reset();
            }

            if (gamepad2.y) {
                isFoldingOut = true;
                outtake.targetPosition = 3;
                tm1.reset();
            }

            if (gamepad2.x && gamepad2.left_bumper) {

            }

            if (isFoldingOut) {

                outtake.setArm(outtake.armPos[outtake.targetPosition], outtake.armPow);

                if (Math.abs(outtake.arm.getCurrentPosition() - outtake.armPos[outtake.targetPosition]) < 30) {
                    outtake.wristOut();
                    isFoldingOut = false;
                }
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.05 && !isFoldingOut) {
                outtake.setArm(outtake.arm.getCurrentPosition() + (int) (5*gamepad2.left_stick_y), outtake.armPow);
            }

            lastRightState = gamepad1.right_bumper;
            lastLeftState = gamepad1.left_bumper;
        }
    }
}