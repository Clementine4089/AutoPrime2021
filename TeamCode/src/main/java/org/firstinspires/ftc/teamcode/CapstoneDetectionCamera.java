package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class CapstoneDetectionCamera {
    WebcamName webcam1;
    OpenCvCamera camera;
    CapstonePipeline pipeline;
    Telemetry telemetry;

    public CapstoneDetectionCamera(HardwareMap hardwareMap) {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1);

        pipeline = new CapstonePipeline();
        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                                   @Override
                                                   public void onOpened() {
                                                       camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                                                       FtcDashboard.getInstance().startCameraStream(camera, 0);
                                                   }

                                                   @Override
                                                   public void onError(int errorCode) {
                                                       System.out.println("error" + errorCode);
                                                       telemetry.addData("Error has occured. Error code - ", errorCode);
                                                   }
                                               }
        );
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