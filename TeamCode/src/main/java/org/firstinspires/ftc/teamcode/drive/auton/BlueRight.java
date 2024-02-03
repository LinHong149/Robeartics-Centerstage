package org.firstinspires.ftc.teamcode.drive.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.pipeline.BlueDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Autonomous(name="BlueRight", group="auton")
public class BlueRight extends LinearOpMode {


    OpenCvCamera webcam1 = null;
    BlueDetectionPipeline blueDetectionPipeline = new BlueDetectionPipeline();

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void runOpMode() {
        // Initialize camera
        initCamera();
        webcam1.setPipeline(blueDetectionPipeline);

        telemetry.addData("Status", "Initialzed");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("position", blueDetectionPipeline.getPosition());
            telemetry.update();
        }
    }





}