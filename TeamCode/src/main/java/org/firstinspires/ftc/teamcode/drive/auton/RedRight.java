package org.firstinspires.ftc.teamcode.drive.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.pipeline.BlueDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="RedRight", group="auton")
public class RedRight extends LinearOpMode {

    // Declare servos and motors
    public Servo wristR = null;
    public Servo armL = null;
    public Servo armR = null;
    public Servo wristL = null;
    public Servo clawR = null;
    public Servo clawL = null;
    public DcMotor fr, fl, br, bl, lSlide, rSlide;
    public Encoder parallelEncoder, perpendicularEncoder;

    boolean clawOpen = false;
    boolean isPickingUp = false;
    boolean isOuttaking = false;

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
    public void setClawLOpen() {
        clawL.setPosition(0.6);
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

//    public void setDriveForward(double distance) {
//        resetEncoders();
//        distance *=100;
//        double startPose = parallelEncoder.getCurrentPosition();
//        while (parallelEncoder.getCurrentPosition() <= startPose+distance) {
////            telemetry.addData("startPose", startPose);
////            telemetry.addData("currPose", parallelEncoder.getCurrentPosition());
////            telemetry.update();
//            fl.setPower(0.7);
//            bl.setPower(0.7);
//            fr.setPower(0.7);
//            br.setPower(0.7);
//        }
//        fl.setPower(0);
//        bl.setPower(0);
//        fr.setPower(0);
//        br.setPower(0);
//    }
public void setDriveForward(int time) {
    fl.setPower(0.7);
    bl.setPower(0.7);
    fr.setPower(0.7);
    br.setPower(0.7);
    sleep(time);
    fl.setPower(0);
    bl.setPower(0);
    fr.setPower(0);
    br.setPower(0);
}
    public void setDriveBackward(int time) {
        fl.setPower(-0.7);
        bl.setPower(-0.7);
        fr.setPower(-0.7);
        br.setPower(-0.7);
        sleep(time);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void setDriveLeft(int time) {
        fl.setPower(-0.7);
        bl.setPower(0.7);
        fr.setPower(0.7);
        br.setPower(-0.7);
        sleep(time);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void setDriveRight(int time) {
        fl.setPower(0.7);
        bl.setPower(-0.7);
        fr.setPower(-0.7);
        br.setPower(0.7);
        sleep(time);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
//    public void setDriveBackward(double distance) {
//        resetEncoders();
//        distance *=100;
//        double startPose = parallelEncoder.getCurrentPosition();
//        while (parallelEncoder.getCurrentPosition() >= startPose-distance) {
//            telemetry.addData("targetPose", startPose-distance);
//            telemetry.addData("currPose", parallelEncoder.getCurrentPosition());
//            telemetry.update();
//            fl.setPower(-0.7);
//            bl.setPower(-0.7);
//            fr.setPower(-0.7);
//            br.setPower(-0.7);
//        }
//        fl.setPower(0);
//        bl.setPower(0);
//        fr.setPower(0);
//        br.setPower(0);
//    }
//    public void setDriveLeft(double distance) {
//        resetEncoders();
//        distance *=100;
//        double startPose = perpendicularEncoder.getCurrentPosition();
//        while (perpendicularEncoder.getCurrentPosition() <= startPose+distance) {
//            telemetry.addData("targetPose", startPose+distance);
//            telemetry.addData("currPose", perpendicularEncoder.getCurrentPosition());
//            telemetry.update();
//            fl.setPower(-0.7);
//            bl.setPower(0.7);
//            fr.setPower(0.7);
//            br.setPower(-0.7);
//        }
//        fl.setPower(0);
//        bl.setPower(0);
//        fr.setPower(0);
//        br.setPower(0);
//    }
//    public void setDriveRight(double distance) {
//        resetEncoders();
//        distance *=100;
//        double startPose = perpendicularEncoder.getCurrentPosition();
//        while (perpendicularEncoder.getCurrentPosition() >= startPose-distance) {
////            telemetry.addData("startPose", startPose);
////            telemetry.addData("currPose", perpendicularEncoder.getCurrentPosition());
////            telemetry.update();
//            fl.setPower(0.7);
//            bl.setPower(-0.7);
//            fr.setPower(-0.7);
//            br.setPower(0.7);
//        }
//        fl.setPower(0);
//        bl.setPower(0);
//        fr.setPower(0);
//        br.setPower(0);
//    }

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
    public void setRotateCounterclockwise(int degrees) {

        double startPose = parallelEncoder.getCurrentPosition();
        while (parallelEncoder.getCurrentPosition() <= startPose+(degrees*90)) {

            telemetry.addData("y", parallelEncoder.getCurrentPosition());
            telemetry.addData("x", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
            fl.setPower(-0.7);
            bl.setPower(-0.7);
            fr.setPower(0.7);
            br.setPower(0.7);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    OpenCvCamera webcam;
    BlueDetectionPipeline pipeline = new BlueDetectionPipeline();
    public BlueDetectionPipeline.Position position = BlueDetectionPipeline.Position.UNKNOWN;

    @Override
    public void runOpMode(){
        initHardware();
        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
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

        telemetry.addData("Status", "Initialzed");

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.addData("position", position);
//        resetEncoders();
        setClawLClose();
        setClawRClose();
        setDriveForward(900);
        wristR.setPosition(0.58);
        wristL.setPosition(0.58);
        armR.setPosition(0);
        armL.setPosition(0);
        sleep(200);
        setClawLOpen();
        setClawROpen();



        telemetry.update();
        sleep(10000);
    }
}