package org.firstinspires.ftc.teamcode.subsystems.centerstage.vision;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class PropSensor {
    private final OpenCvCamera camera;
    public static boolean isOpened = false;
    private final PropSensorPipeline pipeline;
  
    public PropSensor(HardwareMap hardwareMap, boolean isRed) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName name = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(name, cameraMonitorViewId);
  
        pipeline = new PropSensorPipeline(isRed);

        initializeCamera();
    }

    private void initializeCamera() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                isOpened = true;
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public int propPosition() {
        return pipeline.propPosition();
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public boolean getIsOpened() {
        return isOpened;
    }

    public void printTelemetry() {
        mTelemetry.addData("Predicted prop position", propPosition());
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("FPS", camera.getFps());
    }
}
