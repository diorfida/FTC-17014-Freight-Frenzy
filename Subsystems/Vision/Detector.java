package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Detector {

    private LinearOpMode opMode;
    private OpenCvWebcam webcam;
    ContourPipeline myPipeline;

    public Detector(LinearOpMode opMode)
    {
        this.opMode = opMode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        myPipeline = new ContourPipeline();
        webcam.setPipeline(myPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(myPipeline.CAMERA_WIDTH, myPipeline.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public DuckPositionsAndLevels detect()
    {
        if (myPipeline.error)
            opMode.telemetry.addData("Exception: ", myPipeline.debug);

        opMode.telemetry.addData("RectArea: ", myPipeline.getRectArea());
        opMode.telemetry.addLine();

        if(myPipeline.getRectArea() > 2000)
        {
            if(myPipeline.getRectMidpointX() > 400)
            {
                opMode.telemetry.addData("Duck Placement:", DuckPositionsAndLevels.RIGHT_LEVEL3);
                return DuckPositionsAndLevels.RIGHT_LEVEL3;
            }
            else if(myPipeline.getRectMidpointX() > 200)
            {
                opMode.telemetry.addData("Duck Placement:", DuckPositionsAndLevels.MIDDLE_LEVEL2);
                return DuckPositionsAndLevels.MIDDLE_LEVEL2;
            }
            else
            {
                opMode.telemetry.addData("Duck Placement:", DuckPositionsAndLevels.LEFT_LEVEL1);
                return DuckPositionsAndLevels.LEFT_LEVEL1;
            }
        }
        // if something goes wrong...
        return DuckPositionsAndLevels.LEFT_LEVEL1;
    }
}
