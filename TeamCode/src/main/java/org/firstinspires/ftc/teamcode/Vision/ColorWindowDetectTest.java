package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Color Test")
public class ColorWindowDetectTest extends LinearOpMode {

    // Declaring Camera vars
    OpenCvCamera camera;

    // Calibration
    double fx = 1481.603;
    double fy = 1527.539;
    double cx = 550.003;
    double cy = 90.751;

    int r = 10;
    int c = 10;

    int[] dummy = {0};

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ColorPipeline colorPipeline = new ColorPipeline(dummy, dummy, 2);

        camera.setPipeline(colorPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Check Left: ", colorPipeline.checkLeft());
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

        while (opModeIsActive()) {
            //telemetry.addData("Blue at 10, 10:", colorPipeline.getC(10, 10, 0));
            telemetry.addData("Test: ", colorPipeline.processed());

            if (gamepad1.a) {
                telemetry.addData("C: ", colorPipeline.processed().get(10, 10)[0]);
            }
            telemetry.update();
        }
    }
}
