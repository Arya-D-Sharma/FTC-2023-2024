package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CheckMotors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");

        DcMotor winch = hardwareMap.dcMotor.get("WC");

        DcMotor arm = hardwareMap.dcMotor.get("AR");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo drop1 = hardwareMap.servo.get("D1");
        Servo drop2 = hardwareMap.servo.get("D2");
        Servo wrist = hardwareMap.servo.get("WR");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //wrist.setPosition(0.75);

            if (gamepad1.dpad_left) {
                frontLeftMotor.setPower(0.2);
            }

            else {
                frontLeftMotor.setPower(0);
            }

            if (gamepad1.dpad_right) {
                backLeftMotor.setPower(0.2);
            }

            else {
                backLeftMotor.setPower(0);
            }

            if (gamepad1.dpad_up) {
                backRightMotor.setPower(0.2);
            }

            else {
                backRightMotor.setPower(0);
            }

            if (gamepad1.dpad_down) {
                frontRightMotor.setPower(0.2);
            }

            else {
                frontRightMotor.setPower(0);
            }

            if (gamepad2.a) {
                drop1.setPosition(1);
            }

            else {
                drop1.setPosition(0);
            }

            if (gamepad2.b) {
                drop2.setPosition(1);
            }

            else {
                drop2.setPosition(0);
            }

            if (gamepad2.x) {
                wrist.setPosition(1);
            }

            else {
                wrist.setPosition(0.5);
            }

            if (gamepad2.dpad_up) {
                winch.setPower(1);
            }

            else if (gamepad2.dpad_down) {
                winch.setPower(-1);
            }

            else {
                winch.setPower(0);
            }

            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.addData("Drop 1", drop1.getPosition());
            telemetry.addData("Drop 2", drop2.getPosition());
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.update();
        }
    }
}

