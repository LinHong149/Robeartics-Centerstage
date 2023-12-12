package org.firstinspires.ftc.teamcode.drive.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.pipeline.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.drive.pipeline.BlueDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="BlueRight", group="auton")
public class BlueRight extends LinearOpMode {
    // Declare servos and motors
    public Servo wristR = null;
    public Servo armL = null;
    public Servo armR = null;
    public Servo wristL = null;
    public Servo clawR = null;
    public Servo clawL = null;
    public DcMotor fr, fl, br, bl, lSlide, rSlide;
    public Encoder parallelEncoder, perpendicularEncoder;

    double lIntake = 0.77;
    double rIntake = 0.29;
    double lOuttake = 0.16;
    double rOuttake = 0.90;

    // Initialize standard Hardware interfaces
    public void initHardware() {
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "br"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));

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

    public void resetEncoders() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return;
    }
    // Claw
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
    // Arm
    public void setArmDown() {
        wristR.setPosition(0.6);
        wristL.setPosition(0.6);
//        Thread.sleep(2000);
        armL.setPosition(1);
        armR.setPosition(0);
    }
    public void setArmDrive(){
        wristR.setPosition(0.9);
        wristL.setPosition(0.9);
        armL.setPosition(0.9);
        armR.setPosition(0.1);
    }
    // Slide
    public void setSlideExtend(int time) {
        lSlide.setPower(0.4);
        rSlide.setPower(0.4);
        sleep(time);
        lSlide.setPower(0);
        rSlide.setPower(0);
    }
    public void setSlideRetract(int time) {
        lSlide.setPower(-0.4);
        rSlide.setPower(-0.4);
        sleep(time);
        lSlide.setPower(0);
        rSlide.setPower(0);
    }
    // Drive
    public void setDriveForward(double distance) {
        resetEncoders();
        distance *=100;
        double startPose = parallelEncoder.getCurrentPosition();
        while (parallelEncoder.getCurrentPosition() <= startPose+distance) {
//            telemetry.addData("startPose", startPose);
//            telemetry.addData("currPose", parallelEncoder.getCurrentPosition());
//            telemetry.update();
            fl.setPower(0.7);
            bl.setPower(0.7);
            fr.setPower(0.7);
            br.setPower(0.7);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void setDriveBackward(double distance) {
        resetEncoders();
        distance *=100;
        double startPose = parallelEncoder.getCurrentPosition();
        while (parallelEncoder.getCurrentPosition() >= startPose-distance) {
            telemetry.addData("targetPose", startPose-distance);
            telemetry.addData("currPose", parallelEncoder.getCurrentPosition());
            telemetry.update();
            fl.setPower(-0.7);
            bl.setPower(-0.7);
            fr.setPower(-0.7);
            br.setPower(-0.7);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void setDriveLeft(double distance) {
        resetEncoders();
        distance *=100;
        double startPose = perpendicularEncoder.getCurrentPosition();
        while (perpendicularEncoder.getCurrentPosition() <= startPose+distance) {
            telemetry.addData("targetPose", startPose+distance);
            telemetry.addData("currPose", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
            fl.setPower(-0.7);
            bl.setPower(0.7);
            fr.setPower(0.7);
            br.setPower(-0.7);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void setDriveRight(double distance) {
        resetEncoders();
        distance *=100;
        double startPose = perpendicularEncoder.getCurrentPosition();
        while (perpendicularEncoder.getCurrentPosition() >= startPose-distance) {
//            telemetry.addData("startPose", startPose);
//            telemetry.addData("currPose", perpendicularEncoder.getCurrentPosition());
//            telemetry.update();
            fl.setPower(0.7);
            bl.setPower(-0.7);
            fr.setPower(-0.7);
            br.setPower(0.7);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    // Rotate
    public void setRotateClockwise(int degrees) {

        double startPose = parallelEncoder.getCurrentPosition();
        while (parallelEncoder.getCurrentPosition() >= startPose-(degrees*90)) {

            telemetry.addData("y", parallelEncoder.getCurrentPosition());
            telemetry.addData("x", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
            fl.setPower(0.7);
            bl.setPower(0.7);
            fr.setPower(-0.7);
            br.setPower(-0.7);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    // AprilTag Config
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // Units are meters
    double tagsize = 0.166;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    double targetX = -2.00;
    double targetZ = 2.00;
    AprilTagDetection tagOfInterest = null;
    int tagOfInterestId = 2;
    double FEET_PER_METER = 3.28084;

    // Initialize camera
    OpenCvCamera webcam;
    BlueDetectionPipeline propPipeline = new BlueDetectionPipeline();
    AprilTagPipeline aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);
    // Set initial team prop to unknown
    public BlueDetectionPipeline.Position position = BlueDetectionPipeline.Position.UNKNOWN;


    @Override
    public void runOpMode(){
        initHardware();

        // Initialize camera and setting to team prop pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(propPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                position = BlueDetectionPipeline.Position.UNKNOWN;
            }
        });
        telemetry.setMsTransmissionInterval(50);
        position = propPipeline.getPosition();
        webcam.setPipeline(aprilTagPipeline);

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.addData("position", position);

        if (position == BlueDetectionPipeline.Position.LEFT  || position == BlueDetectionPipeline.Position.UNKNOWN) {
            telemetry.addData("Status", "Running if pose unknown");
            // setDriveRight
            // setDriveBackward
            // setRotateCounterclockwise
            // setArmDown
            // setClawLOpen
            // setArmDrive
            tagOfInterestId = 1;
            while (!isStopRequested()) {
                findAprilTag();
                telemetry.addData("tagFound", tagOfInterest);
                if (tagOfInterest != null) {
                    tagToTelemetry(tagOfInterest);
                    if (tagOfInterest.pose.x*FEET_PER_METER > targetX + 0.5) {
                        telemetry.addData("move ", "left");
                    } else if (tagOfInterest.pose.x*FEET_PER_METER < targetX - 0.5) {
                        telemetry.addData("move ", "right");
                    } else if (tagOfInterest.pose.z*FEET_PER_METER > targetZ) {
                         telemetry.addData("move ", "forward");
                    }
                }
            telemetry.update();
            }

        }
        else if (position == BlueDetectionPipeline.Position.CENTER) tagOfInterestId = 2;
        else if (position == BlueDetectionPipeline.Position.RIGHT) tagOfInterestId = 3;




        telemetry.update();
        sleep(30000);
    }
    void findAprilTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();
        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == tagOfInterestId) {
                    tagOfInterest = tag;
                    tagFound = true;
                    telemetry.addData("findAprilTag() tagOfInterest ", tagOfInterest);
                    break;
                }
            }

            if(tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
        }
        else {
            telemetry.addLine("Don't see tag of interest :( size");

            if(tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }
        }
        sleep(50);
    }
    void tagToTelemetry(AprilTagDetection detection) {
        if (detection != null) {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

        }
    }
}