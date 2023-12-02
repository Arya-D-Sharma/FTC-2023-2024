package org.firstinspires.ftc.teamcode.Teleops.FinalCleaned;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeHandler {

    // Vars for motor declaration
    DcMotorEx belts;
    DcMotorEx tubes;

    // Vars for running
    double runPow = 0;

    public IntakeHandler (HardwareMap hardwareMap) {
        belts = hardwareMap.get(DcMotorEx.class, "BE");
        tubes = hardwareMap.get(DcMotorEx.class, "IN");

        tubes.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void forward(double p) {
        belts.setPower(p);
        tubes.setPower(p);
    }

    public void backward(double p) {
        belts.setPower(-p);
        tubes.setPower(-p);
    }

    public void stop() {
        tubes.setPower(0);
        belts.setPower(0);
    }
}
