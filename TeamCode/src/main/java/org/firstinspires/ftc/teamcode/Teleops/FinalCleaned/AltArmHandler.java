package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AltArmHandler {

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

    public boolean isPlacing = false;
    public boolean folding = false;

    public int targetPosition = 0;
    public double wristTarget;

    int armBack = 0;
    int[] armOut = new int[] {0, 689, 1118, 1547};
    double armPow = 0.5;
    double slowPow = 0.35;

    ElapsedTime tm;

    public AltArmHandler (HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "AR");
        wrist = hardwareMap.get(Servo.class, "WR");
        drop1 = hardwareMap.get(Servo.class, "D1");
        drop2 = hardwareMap.get(Servo.class, "D2");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist.setPosition(wristIn);
        drop1.setPosition(placeAngle);
        drop2.setPosition(1-placeAngle);

        tm = new ElapsedTime();
        tm.startTime();
    }

    // Arm declarations
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

    public void updateArm() {

        if (targetPosition == 0) {
            arm.setTargetPosition(0);

            wristIn();
            tm.reset();
            double time = 0;
            while (time < 0.500) {
                wristIn();
                time = tm.time();
            }                                                   
        }

        else if (Math.abs(arm.getTargetPosition() - armOut[targetPosition]) > 20) {

            wristIn();
            tm.reset();
            double time = 0;
            while (time < 0.500) {
                wristIn();
                time = tm.time();
            }
            
            d1 = false;
            d2 = false;
            dropUpdate();
        }

        else {
            wristOut();
        }

        setArm(armOut[targetPosition], armPow);
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
