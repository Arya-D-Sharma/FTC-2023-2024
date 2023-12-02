package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Wrist")
public class WristSandbox extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors

        Servo wrist = hardwareMap.servo.get("WR");
        DcMotor arm = hardwareMap.dcMotor.get("AR");
        int pos = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                wrist.setPosition(0.5);
            }

            if (gamepad1.b) {
                wrist.setPosition(0.6);
            }

            if (gamepad1.x) {
                wrist.setPosition(0.4);
            }

            if (gamepad1.y) {
                wrist.setPosition(0.74);
            }


            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}

