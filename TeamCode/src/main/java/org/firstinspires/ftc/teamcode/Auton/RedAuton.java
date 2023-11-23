package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipeline.RedDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="RedAuton", group="Linear Opmode")
public class RedAuton extends LinearOpMode {
    OpenCvCamera webcam;
    RedDetectionPipeline pipeline = new RedDetectionPipeline();

    @Override
    public void runOpMode() {
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline
        webcam.setPipeline(pipeline);

        // Start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera open error
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            RedDetectionPipeline.Position position = pipeline.getPosition();
            telemetry.addData("positionGiven", pipeline.getPosition().toString());

            // Use the position data to control the robot
            switch(position) {
                case LEFT:
                    // Object is on the left
                    telemetry.addData("position", "left");
                    break;
                case CENTER:
                    // Object is in the center
                    telemetry.addData("position", "center");
                    break;
                case RIGHT:
                    // Object is on the right
                    telemetry.addData("position", "right");
                    break;
                case UNKNOWN:
                    // Object position is unknown
                    telemetry.addData("position", "unknown");
                    break;
            }
            telemetry.update();

            // Your OpMode logic
            sleep(50);
        }
    }
}
