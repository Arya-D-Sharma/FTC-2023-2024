package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FinalArm {

    // Arm motors
    public DcMotorEx arm;
    public Servo wrist;
    public Servo drop1;
    public Servo drop2;


    // Arms Vars
    double closeAngle = 1;
    double placeAngle = 0;

    public boolean d1 = false;
    public boolean d2 = false;

    double wristIn = 0.45;
    double wristOut = 0.74;

    public int targetPosition = 0;
    public double wristTarget;

    public int[] armPos = new int[]{0, 689, 1118, 1547};
    public double armPow = 0.8;
    public double slowPow = 0.6;

    ElapsedTime tm;

    public FinalArm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "AR");
        wrist = hardwareMap.get(Servo.class, "WR");
        drop1 = hardwareMap.get(Servo.class, "D1");
        drop2 = hardwareMap.get(Servo.class, "D2");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist.setPosition(wristIn);
        drop1.setPosition(placeAngle);
        drop2.setPosition(1 - placeAngle);

        tm = new ElapsedTime();
        tm.startTime();
    }

    public void setArm(int pos, double pow) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(pow);
    }

    public void dropUpdate() {
        if (d1) {
            drop1.setPosition(placeAngle);
        }

        else {
            drop1.setPosition(closeAngle);
        }

        if (d2) {
            drop2.setPosition(1-placeAngle);
        }

        else {
            drop2.setPosition(1-closeAngle);
        }
    }

    public void wristOut() {
        wrist.setPosition(wristOut);
        wristTarget = wristOut;
    }

    public void wristIn() {
        wrist.setPosition(wristIn);
        wristTarget = wristIn;
    }
}