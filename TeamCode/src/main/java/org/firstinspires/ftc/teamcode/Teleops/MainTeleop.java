package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class MainTeleop extends LinearOpMode {

    // Motor Declarations

    // Drive Motors
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx fr;
    DcMotorEx fl;

    // Arm motors
    DcMotorEx arm;
    Servo wrist;
    Servo drop1;
    Servo drop2;

    // Arms Vars
    double closeAngle;
    double placeAngle;

    // Intake Motors
    DcMotorEx belts;
    DcMotorEx tubes;

    // Intake vars
    double maxInPower = 0.5;

    // Arm declarations
    public void setArm(int pos) {
        arm.setTargetPosition(pos);
    }

    public void setWrist(double val) {
        wrist.setPosition(val);
    }

    public void dropHandler(boolean serv1, boolean serv2) {
        if (serv1) {
            drop1.setPosition(placeAngle);
        }

        else {
            drop1.setPosition(closeAngle);
        }

        if (serv2) {
            drop2.setPosition(placeAngle);
        }

        else {
            drop2.setPosition(closeAngle);
        }
    }

    // Intake Declarations
    public void intake(int status) { // 0: Stop, 1:Intake, 2:Reverse

        if (status == 1) {
            belts.setDirection(DcMotorSimple.Direction.FORWARD);
            tubes.setDirection(DcMotorSimple.Direction.FORWARD);
            belts.setPower(maxInPower);
            tubes.setPower(maxInPower);
        }

        else if (status == 2) {
            belts.setDirection(DcMotorSimple.Direction.REVERSE);
            tubes.setDirection(DcMotorSimple.Direction.REVERSE);
            belts.setPower(maxInPower);
            tubes.setPower(maxInPower);
        }

        else {
            belts.setPower(0);
            tubes.setPower(0);
        }
    }

    @Override
    public void runOpMode() {

        // Test intake
        if (gamepad1.right_trigger > 0.25) {
            intake(1);
        }

        else if (gamepad1.left_trigger > 0.25) {
            intake(2);
        }

        else {
            intake(0);
        }

        // Test Outtake

    }
}
