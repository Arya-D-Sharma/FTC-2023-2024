package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.MecanumDriveHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.IntakeHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.EndgameHandler;
import org.firstinspires.ftc.teamcode.Teleops.FinalCleaned.ArmHandler;

@TeleOp(name = "0A Greenbeard's Grey Matter")
public class GrayMatter extends LinearOpMode {

    // Drive Vars
    boolean driveAllowed = true;

    boolean heldHeading = false;
    boolean isRed = true;

    int cancelState = 0;

    boolean lastStart = false;

    double xMove;
    double yMove;
    double rX ;
    double angle;
    double angleCorrection = Math.PI/2;
    boolean lastRightState = false;
    boolean lastLeftState = false;

    int targetPosition = 0;
    boolean isFoldingOut = false;
    boolean isFoldingIn = false;

    boolean lastY;
    boolean lastB;
    double multiplier = 1;

    int holdvar = 0;
    boolean holding = false;

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
            xMove = gamepad1.left_stick_x;
            yMove = -gamepad1.left_stick_y;
            rX = gamepad1.right_stick_x;

            if (Math.abs(rX) > 0.05) {
                heldHeading = false;
                multiplier = 1;
            }

            if (driveAllowed) {
                if (heldHeading) {
                    angle = drive.getImu() - angleCorrection;
                    if (angle >= Math.PI){
                        angle -= 2*Math.PI;
                    }
                    if (angle <= -Math.PI){
                        angle += 2*Math.PI;
                    }

                    drive.run(xMove, yMove, 2. * angle, multiplier);

                    /*
                    double sign = 1.0;

                    if (drive.getImu() > 0) {
                        sign = 1.0;
                    }
                    else {
                        sign = -1.0;
                    }

                    if (isRed) {

                        if (drive.getImu() - sign*Math.PI*0.5 > 0) {
                            drive.run(xMove, yMove, rX + 2*Math.abs(Math.PI*0.5-drive.getImu()), multiplier);
                        }

                        else {
                            drive.run(xMove, yMove, rX - 2*Math.abs(Math.PI*0.5-drive.getImu()), multiplier);
                        }
                    }

                    else {
                        if (drive.getImu() + sign*Math.PI*0.5 > 0) {
                            drive.run(xMove, yMove, rX + 2*Math.abs(Math.PI*0.5-drive.getImu()), multiplier);
                        }

                        else {
                            drive.run(xMove, yMove, rX - 2*Math.abs(Math.PI*0.5-drive.getImu()), multiplier);
                        }
                    }
                    */

                }

                else {
                    drive.run(xMove, yMove, rX, multiplier);
                }
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
                    holding = false;
                } else if (gamepad2.dpad_down) {
                    end.winchDown();
                    holding = false;
                } else if (gamepad2.dpad_left) {
                    holdvar = end.winch.getCurrentPosition();
                    holding = true;
                }
                else if (holding){
                    end.winchHold(holdvar);
                }
                else {
                    end.winch.setPower(0);
                }
                if (gamepad2.left_bumper) {
                    end.launch();
                }
                if (gamepad2.right_bumper) {
                    outtake.setArm(200, outtake.armPow);
                    end.extendRod();
                }
            }

            if (gamepad1.left_bumper && !lastLeftState && !isFoldingOut) {
                outtake.d1 = !outtake.d1;
                outtake.dropUpdate();
            }

            if (gamepad1.right_bumper && !lastRightState && !isFoldingOut) {
                outtake.d2 = !outtake.d2;
                outtake.dropUpdate();
            }

            if (gamepad1.start && !lastStart) {
                isRed = !isRed;
                angleCorrection *= -1;
            }

            if (gamepad1.y && !lastY) {
                heldHeading = !heldHeading;
            }

            if (gamepad1.b) {
                multiplier = 0.5;
            }

            else {
                multiplier = 1;
            }

            if (gamepad2.a) {
                isFoldingOut = true;

                outtake.targetPosition = 1;

                if (gamepad2.left_trigger > 0.05) {
                    outtake.targetPosition = 4;
                }

                outtake.d1 = false;
                outtake.d2 = false;
                outtake.dropUpdate();

                tm1.reset();
            }

            if (gamepad2.b) {
                isFoldingOut = true;
                outtake.targetPosition = 2;

                if (gamepad2.left_trigger > 0.05) {
                    outtake.targetPosition = 5;
                }

                outtake.d1 = false;
                outtake.d2 = false;
                outtake.dropUpdate();

                tm1.reset();
            }

            if (gamepad2.y) {
                isFoldingOut = true;
                outtake.targetPosition = 3;

                if (gamepad2.left_trigger > 0.05) {
                    outtake.targetPosition = 6;
                }

                outtake.d1 = false;
                outtake.d2 = false;
                outtake.dropUpdate();

                tm1.reset();
            }

            if (gamepad2.x && gamepad2.right_bumper) {
                isFoldingIn = true;
                isFoldingOut = false;
                cancelState = 0;
                outtake.setArm(outtake.armPos[outtake.targetPosition] + 300, outtake.slowPow);
            }

            if (isFoldingOut) {

                outtake.setArm(outtake.armPos[outtake.targetPosition], outtake.armPow);

                if (Math.abs(outtake.arm.getCurrentPosition() - outtake.armPos[outtake.targetPosition]) < 30) {
                    outtake.wristOut();
                    isFoldingOut = false;
                }
            }

            if (isFoldingIn) {

                if (Math.abs(outtake.arm.getCurrentPosition() - outtake.armPos[outtake.targetPosition] - 300) < 20 && cancelState == 0) {
                    outtake.wristIn();
                    cancelState = 1;
                    tm1.reset();
                }

                if (tm1.milliseconds() > 500 && cancelState == 1) {
                    outtake.targetPosition = 0;
                    outtake.setArm(outtake.targetPosition, outtake.slowPow);
                    isFoldingIn = false;
                    cancelState = 0;
                    // Changes here
                    //outtake.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    //outtake.arm.setPower(0);
                }
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.05 && !isFoldingOut) {
                outtake.setArm(outtake.arm.getCurrentPosition() - (int) (20*gamepad2.left_stick_y), outtake.armPow);
                //outtake.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //outtake.arm.setPower(0.5*gamepad2.left_stick_y);
            }

            if (outtake.arm.getTargetPosition() < 0) {
                outtake.setArm(0, outtake.slowPow);
            }

            if (outtake.arm.getTargetPosition() == 0 && outtake.arm.getCurrentPosition() == 0) {
                outtake.arm.setPower(0);
            }

            lastRightState = gamepad1.right_bumper;
            lastLeftState = gamepad1.left_bumper;
            lastY = gamepad1.y;
            lastB = gamepad1.b;
            lastStart = gamepad1.start;

            telemetry.addData("Orientation", drive.getImu());
            telemetry.addData("Corrected Angle", angle);
            telemetry.addData("Arm", outtake.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}