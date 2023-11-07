package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Color Auto")
public class ColorAutoTest extends LinearOpMode {

    OpenCvCamera camera;
    int detection = 2;

    int[] leftCoords = {10, 15, 10, 20}; // x1, x2, y1, y2
    int[] centerCoords = {30, 35, 10, 20}; // x1, x2, y1, y2

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ColorPipeline colorPipeline = new ColorPipeline(leftCoords, centerCoords, 200);

        camera.setPipeline(colorPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Check Left: ", colorPipeline.checkLeft());
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.addData("Test: ", colorPipeline.processed());
        telemetry.update();

        waitForStart();

        //telemetry.addData("Left");
    }
}
