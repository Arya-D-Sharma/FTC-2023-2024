package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Gray Matter")
public class MainTeleop extends LinearOpMode {

    // Timer
    ElapsedTime tm1 = new ElapsedTime();

    // IMU
    BNO055IMU imu;
    Orientation orientation;

    // Motor Declarations

    // Drive Motors
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx fr;
    DcMotorEx fl;

    double driveSpeed = 0.5;
    double turningSpeed = 0.5;

    boolean driveAllowed = true;

    double y_move;
    double x_move;
    double rotation_x;

    // Arm motors
    DcMotorEx arm;
    Servo wrist;
    Servo drop1;
    Servo drop2;

    Servo planeTrigger;
    double launchAngle = 0.5;

    // Arms Vars
    double closeAngle = 1;
    double placeAngle = 0;

    double wristIn = 0.45;
    double wristOut = 0.75;

    boolean foldingBack = false;
    boolean foldingOut = false;

    int armBack = 0;
    int armOut = 1000;
    int armVel = 800;

    // Intake Motors
    DcMotorEx belts;
    DcMotorEx tubes;

    // Intake vars
    double maxInPower = 1;

    // Arm declarations
    public void setArm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVel);
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
            drop2.setPosition(1-placeAngle);
        }

        else {
            drop2.setPosition(1-closeAngle);
        }
    }

    // Intake Declarations
    public void intake(int status) { // 0: Stop, 1:Intake, 2:Reverse

        if (status == 1) {
            belts.setPower(-maxInPower);
            tubes.setPower(maxInPower);
        }

        else if (status == 2) {
            belts.setPower(maxInPower);
            tubes.setPower(-maxInPower);
        }

        else {
            belts.setPower(0);
            tubes.setPower(0);
        }
    }

    public void FCDrive(double x_move, double y_move, double rotation_x) {

        if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
            driveSpeed = 1.0;
            turningSpeed = 1.0;
        }

        else {
            driveSpeed = 0.5;
            turningSpeed = 0.7;
        }

        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double x = x_move * Math.cos(-(orientation.firstAngle)) - y_move * Math.sin(-(orientation.firstAngle));
        double y = y_move * Math.cos(-(orientation.firstAngle)) + x_move * Math.sin(-(orientation.firstAngle));

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation_x), 1);

        double frontLeftPower = (y + x + rotation_x) / denominator;
        double backLeftPower = (y - x + rotation_x) / denominator;
        double frontRightPower = (y - x - rotation_x) / denominator;
        double backRightPower = (y + x - rotation_x) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        telemetry.addData("Orientation ", orientation);
        telemetry.addData("First Angle ", orientation.firstAngle);
        telemetry.update();

    }

    public void Outtake () {

        setArm(armOut);

        if (gamepad2.x && gamepad2.right_bumper) {
            foldingOut = false;
            foldingBack = true;
            return;
        }

        if ( Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) < 50) {
            wrist.setPosition(wristOut);

            dropHandler(gamepad2.x, gamepad2.b);
        }

    }

    public void foldBack () {

        // Folding out to false
        foldingOut = false;

        // Set wrist back in
        wrist.setPosition(wristIn);

        if (tm1.time() > 0.5) {
            setArm(armBack);
        }

        if ( Math.abs(arm.getCurrentPosition() - armBack) < 20) {
            dropHandler(true, true);
            foldingBack = false;
        }
    }

    public void launch () {
        planeTrigger.setPosition(launchAngle);
    }

    @Override
    public void runOpMode() {

        // IMU Initializations
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Motor Initializations
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        br = hardwareMap.get(DcMotorEx.class, "BR");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        belts = hardwareMap.get(DcMotorEx.class, "BE");
        tubes = hardwareMap.get(DcMotorEx.class, "IN");

        belts.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm
        arm = hardwareMap.get(DcMotorEx.class, "AR");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        drop1 = hardwareMap.get(Servo.class, "D1");
        drop2 = hardwareMap.get(Servo.class, "D2");
        wrist = hardwareMap.get(Servo.class, "WR");
        // Servo 0, 1, 2
        // Arm: Extention spot 3

        waitForStart();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tm1.startTime();

        while(opModeIsActive()) {

            y_move = -gamepad1.left_stick_y;
            x_move = gamepad1.left_stick_x;

            rotation_x = gamepad1.right_stick_x * 1.1;

            driveAllowed = y_move > 0.05 || x_move > 0.05 ||rotation_x > 0.05 ||y_move < -0.05 || x_move < -0.05 ||rotation_x < -0.05;

            if (driveAllowed) {
                FCDrive(x_move, y_move, rotation_x);
            }

            else {
                br.setPower(0);
                bl.setPower(0);
                fl.setPower(0);
                fr.setPower(0);
            }


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

            if (gamepad2.left_bumper && gamepad2.y && !foldingBack) {
                tm1.reset();
                foldingBack = true;
            }

            if (foldingBack) {
                foldBack();
            }

            if (gamepad2.left_bumper) {
                foldingOut = true;
                dropHandler(false, false);
                Outtake();
            }

            if (foldingOut) {
                Outtake();
            }

            telemetry.addData("Arm ", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
