package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class CapstoneDetectionCamera {
    WebcamName webcamName;
    int cameraMonitorViewId;
    OpenCvCamera camera;

    CapstonePipeline pipeline;

    public CapstoneDetectionCamera(HardwareMap hardwareMap, boolean useWebCam2) {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new CapstonePipeline();
        camera.setPipeline(pipeline);
    }

    public CapstonePipeline.CapstonePosition getPosition()
    {
        return pipeline.position;
    }

    public int[] getAnalysis()
    {
        return pipeline.getAnalysis();
    }
}