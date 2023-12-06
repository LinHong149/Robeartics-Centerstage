package org.firstinspires.ftc.teamcode.drive.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.pipeline.BlueDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Autonomous(name="BlueAuton", group="auton")
public class BlueAuton extends LinearOpMode {
    OpenCvCamera webcam;
    BlueDetectionPipeline pipeline = new BlueDetectionPipeline();

    // Declare servos and motors
    public Servo wristR = null;
    public Servo armL = null;
    public Servo armR = null;
    public Servo wristL = null;
    public Servo clawR = null;
    public Servo clawL = null;
    public DcMotor fr, fl, br, bl, lSlide, rSlide;

    boolean clawOpen = false;
    boolean isPickingUp = false;
    boolean isOuttaking = false;

    double lIntake = 0.77;
    double rIntake = 0.29;
    double lOuttake = 0.16;
    double rOuttake = 0.90;

    // Initialize standard Hardware interfaces
    public void initHardware() {
        // Servos
        wristR = hardwareMap.get(Servo.class, "wristR");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        clawL = hardwareMap.get(Servo.class, "clawL");

        //--------------------------------------------------MAYBE IMPORTANT---------------------
        // armL.setPosition(1); //towards 0 is clockwise
        // armR.setPosition(0); //towards 1 is clockwise +0.25 = armL
        // wristR.setPosition(0.85);
        // wristL.setPosition(0.85); //both the same 0 is counterclockwise
        // clawR.setPosition(0.47);
        // clawL.setPosition(0.62);

        // Motors
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        lSlide = hardwareMap.get(DcMotor.class, "lSlide");
        rSlide = hardwareMap.get(DcMotor.class, "rSlide");

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        lSlide.setPower(0);
        rSlide.setPower(0);

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        lSlide.setDirection(DcMotor.Direction.REVERSE);
        rSlide.setDirection(DcMotor.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setClawLOpen() {
        clawL.setPosition(0.7);
    }
    public void setClawLClose() {
        clawL.setPosition(0.43);
    }
    public void setClawROpen() {
        clawR.setPosition(0.6);
    }
    public void setClawRClose() {
        clawR.setPosition(0.33);
    }
    public void setArmPickUp() {
        wristR.setPosition(0.6);
        wristL.setPosition(0.6);
        armL.setPosition(1);
        armR.setPosition(0);
    }
    public void setArmDrive(){
        wristR.setPosition(0.9);
        wristL.setPosition(0.9);
        armL.setPosition(0.9);
        armR.setPosition(0.1);
    }

    @Override
    public void runOpMode() {
        // Initialize Camera
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
                BlueDetectionPipeline.Position position = BlueDetectionPipeline.Position.UNKNOWN;
            }
        });


        initHardware();
        telemetry.addData("Status", "Initialized");


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            BlueDetectionPipeline.Position position = pipeline.getPosition(); // might need some time
            webcam.stopStreaming(); // To stop the position from changing
            telemetry.addData("positionGiven", pipeline.getPosition().toString());

            // Use the position data to control the robot
            switch (position) {
                case LEFT:
                    telemetry.addData("position", "left");
                    setClawLClose();
                    break;
                case CENTER:
                    telemetry.addData("position", "center");
                    setClawRClose();
                    break;
                case RIGHT:
                    telemetry.addData("position", "right");
                    setClawRClose();
                    break;
                case UNKNOWN:
                    telemetry.addData("position", "unknown");
                    break;
            }
            telemetry.update();

            // Your OpMode logic
            sleep(50);


            telemetry.update();
        }
    }
}
