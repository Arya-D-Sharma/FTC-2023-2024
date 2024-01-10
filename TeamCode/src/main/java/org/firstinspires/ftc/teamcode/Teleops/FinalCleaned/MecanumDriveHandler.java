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

public class MecanumDriveHandler {

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

    // Updating variables
    double y_move;
    double x_move;
    double rotation_x;

    // Getting Hardware map from main file and setting up
    public MecanumDriveHandler(HardwareMap hardwareMap) {

        // Motor definitions
        br = hardwareMap.get(DcMotorEx.class, "BR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        fl = hardwareMap.get(DcMotorEx.class, "FL");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void run(double x_move, double y_move, double rotation_x, double pow) {

        if (Math.abs(x_move) > 0.05 || Math.abs(y_move) > 0.05 || Math.abs(rotation_x) > 0.05) {
            // Orientation
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            // Calculating power variables
            double x = x_move * Math.cos(-(orientation.firstAngle)) - y_move * Math.sin(-(orientation.firstAngle));
            double y = y_move * Math.cos(-(orientation.firstAngle)) + x_move * Math.sin(-(orientation.firstAngle));

            // Dividing power by a denominator to set maximum output to 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation_x), 1);

            double frontLeftPower = (y + x + rotation_x) / denominator;
            double backLeftPower = (y - x + rotation_x) / denominator;
            double frontRightPower = (y - x - rotation_x) / denominator;
            double backRightPower = (y + x - rotation_x) / denominator;

            // Writing the power to the motors
            fl.setPower(frontLeftPower*pow);
            bl.setPower(backLeftPower*pow);
            fr.setPower(frontRightPower*pow);
            br.setPower(backRightPower*pow);
        }

        else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
    }

    public double getImu() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

}
