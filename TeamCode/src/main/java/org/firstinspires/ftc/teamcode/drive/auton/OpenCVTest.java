//package org.firstinspires.ftc.teamcode.drive.auton;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.pipeline.BlueDetectionPipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//
//@Autonomous(name="OpenCVTest", group="auton")
//public class OpenCVTest extends LinearOpMode {
//    OpenCvCamera webcam;
//    BlueDetectionPipeline blueDetectionPipeline = new BlueDetectionPipeline();
//
//    @Override
//    public void runOpMode() {
//        // Initialize the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // Set the pipeline
//        webcam.setPipeline(blueDetectionPipeline);
//
//        // Start streaming
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                // Handle camera open error
//            }
//        });
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            BlueDetectionPipeline.Position position = blueDetectionPipeline.getPosition();
//            telemetry.addData("positionGiven", blueDetectionPipeline.getPosition().toString());
//
//            // Use the position data to control the robot
//            switch(position) {
//                case LEFT:
//                    // Object is on the left
//                    telemetry.addData("position", "left");
//                    break;
//                case MIDDLE:
//                    // Object is in the center
//                    telemetry.addData("position", "center");
//                    break;
//                case RIGHT:
//                    // Object is on the right
//                    telemetry.addData("position", "right");
//                    break;
//            }
//            telemetry.update();
//
//            // Your OpMode logic
//            sleep(50);
//        }
//    }
//}
