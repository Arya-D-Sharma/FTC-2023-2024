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

public class ArmHandler {

    // Timer
    ElapsedTime tm1 = new ElapsedTime();

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

    boolean foldingBack = false;
    boolean foldingOut = false;

    public boolean placing = false;

    int armBack = 0;
    int[] armOut = new int[] {0, 689, 1118, 1547};
    public double armPow = 0.5;
    public double slowPow = 0.35;

    public ArmHandler (HardwareMap hardwareMap) {
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

    public void place(int index) {

        placing = true;
        d1 = false;
        d2 = false;
        dropUpdate();

        tm1.reset();
        setArm(armOut[index], armPow);

        while (Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) > 30) {
            setArm(armOut[index], armPow);
        }

        wristOut();

    }

    public void foldBack() {

        placing = false;
        tm1.reset();

        wristIn();

        while(tm1.time() < 0.5) {
            wristIn();
        }

        setArm(0, slowPow);

        while(Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) > 30) {
            d1 = false;
            d2 = false;
            dropUpdate();
        }

        d1 = true;
        d2 = true;
        dropUpdate();
    }

    public void wristOut() {
        wrist.setPosition(wristOut);
    }

    public void wristIn() {
        wrist.setPosition(wristIn);
    }
}
