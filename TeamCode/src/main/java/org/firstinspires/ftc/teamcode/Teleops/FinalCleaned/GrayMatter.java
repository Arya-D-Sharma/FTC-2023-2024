package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public void runOpMode() {

        waitForStart();

        MecanumDriveHandler drive = new MecanumDriveHandler(hardwareMap);
        telemetry.addLine("Mecanum Handler Good");
        IntakeHandler intake = new IntakeHandler(hardwareMap);
        telemetry.addLine("Intake Handler Good");
        EndgameHandler end = new EndgameHandler(hardwareMap);
        telemetry.addLine("Endgame Handler Good");
        ArmHandler outtake = new ArmHandler(hardwareMap);
        telemetry.addLine("Arm Handler Good");
        telemetry.update();

        while(opModeIsActive()) {
            // Intake Runner
            if (gamepad1.right_trigger > 0.15) {
                intake.forward(gamepad1.right_trigger);
                outtake.dropHandler(true, true);
            }

            else if (gamepad1.left_trigger > 0.15) {
                intake.backward(gamepad1.left_trigger);
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
                } else if (gamepad2.dpad_left) {
                    end.winchHold();
                }

                if (gamepad2.left_bumper) {
                    end.launch();
                }

                if (gamepad2.right_bumper) {
                    end.extendRod();
                }

            }

            if (gamepad2.a && !outtake.placing) {
                outtake.place(0);
            }

            else if (gamepad2.b && !outtake.placing) {
                outtake.place(1);
            }

            else if (gamepad2.y && !outtake.placing) {
                outtake.place(2);
            }

            else if (gamepad2.x && gamepad2.right_bumper) {
                outtake.foldBack();
            }

            if (gamepad2.left_bumper) {
                outtake.dropHandler(false, false);
            }

            if (gamepad2.left_trigger > 0.05) {
                outtake.dropHandler(gamepad2.x, gamepad2.y);
            }

            // Edge detection for toggle
        }
    }
}
