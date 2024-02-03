package org.firstinspires.ftc.teamcode.drive.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.pipeline.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.drive.pipeline.BlueDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Autonomous(name="PipelineTesting", group="auton")
public class PipelineTesting extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    OpenCvCamera webcam1 = null;

    // Pipelines
    BlueDetectionPipeline blueDetectionPipeline = new BlueDetectionPipeline();
    AprilTagPipeline aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

//        webcam1.setPipeline(blueDetectionPipeline);
        webcam1.setPipeline(aprilTagPipeline);

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
        initCamera();

        telemetry.addData("Status", "Initialzed");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
//            telemetry.addData("position", blueDetectionPipeline.getPosition());

            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

            for(AprilTagDetection tag : currentDetections) {
                tagToTelemetry(tag);
            }


            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }





}