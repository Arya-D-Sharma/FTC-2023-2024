package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EndgameHandler {

    public boolean active = false;

    Servo planeLauncher;
    Servo telescopeOpener;

    double releaseAngle = 0.7;
    double lockAngle = 0.5;
    double telescopeAngle = 0;
    double holdAngle = 0.5;

    DcMotorEx winch;

    boolean winchAllowed = false;

    public EndgameHandler(HardwareMap hardwareMap) {
        winch = hardwareMap.get(DcMotorEx.class, "WC");

        planeLauncher = hardwareMap.get(Servo.class, "AP");
        telescopeOpener = hardwareMap.get(Servo.class, "TE");

        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        planeLauncher.setPosition(lockAngle);
        telescopeOpener.setPosition(holdAngle);
    }

    public void activate() {
        active = true;
    }

    public void deactive () {
        active = false;
    }

    public void winchUp () {
        if (winchAllowed) {
            winch.setPower(1);
            winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void winchDown() {
        if (winchAllowed) {
            winch.setPower(-1);
            winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void winchHold() {
        if (winchAllowed) {
            winch.setPower(0);
            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            winch.setTargetPosition(winch.getCurrentPosition());
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winch.setVelocity(400);
        }
    }

    public void launch() {
        planeLauncher.setPosition(releaseAngle);
    }

    public void extendRod() {
        telescopeOpener.setPosition(telescopeAngle);
        winchAllowed = true;
    }
}