package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
@TeleOp(name="MecanumTeleop", group="Mecanum")
public class MecanumTeleop extends LinearOpMode {

    // Declare servos and motors
    public Servo clawL = null, clawR = null, wristL = null, wristR = null;
    public DcMotor arm, slide, fl, fr, bl, br;

    double clawLOpen = 0.1;
    double clawROpen = 0.9;
    double clawLClose = 0.44;
    double clawRClose = 0.60;
    boolean clawOpen = false;
    boolean aPreviouslyPressed = false;

    // double wristLOuttake = 0;
    // double wristROuttake = 0.96;
    // double wristLOuttake2 = 0+0.17;
    // double wristROuttake2 = 0.96-0.17;
    double armUpperLimit = -1534;
    double armLowerLimit = -1773;
    // double wristLOuttake = 0;
    // double wristROuttake = 0;
    double currArmPose = 0;
    double adjustmentMultiplier = 0.17/-239;
    double adjustmentFactor = 0;
    double wristLParallel = 0.96-0.05;
    double wristRParallel = 0+0.05;
    boolean wristParallel = false;
    boolean bPreviouslyPressed = false;


    // Arm pidf
    public static PIDFController armPIDF = new PIDFController(0,0,0,0);
    public static double armP = 0, armI = 0, armD = 0, armF = 0;
    public static int armTarget = 0;
    // Slides pid
    public static PIDController slidePID = new PIDController(0,0,0);
    public static double slideP = 0.045, slideI = 0, slideD = 0;
    public static int slideTarget = 670;


    // Initialize standard Hardware interfaces
    public void initHardware() {
        // Servos
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");

        clawL.setPosition(0.44);
        clawR.setPosition(0.60);
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");

        // initialize vertically
        wristL.setPosition(0+0.35);
        // wristR.setPosition(0.96-0.35);

        // Motors
        arm = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        arm.setPower(0);
        slide.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        arm.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");


            // Motors: driving
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);

            double frontLeftPower = (drive + strafe + rotate) / denominator * 0.8;
            double backLeftPower = (drive - strafe + rotate) / denominator * 0.8;
            double frontRightPower = (drive - strafe - rotate) / denominator * 0.8;
            double backRightPower = (drive + strafe - rotate) / denominator * 0.8;

            fr.setPower(frontRightPower);
            fl.setPower(frontLeftPower);
            br.setPower(backRightPower);
            bl.setPower(backLeftPower);

            telemetry.addData("strafe","%.2f", strafe);
            telemetry.addData("drive","%.2f", drive);
            telemetry.addData("rotate","%.2f", rotate);




            // //arm
            // if(gamepad1.a){
            //     arm.setPower(0.6);
            // }
            // else if (gamepad1.b){
            //     arm.setPower(-0.6);
            // }
            // else {
            //     arm.setPower(0);
            // }

            // //slide
            // if(gamepad1.x){
            //     slide.setPower(0.6);
            // }
            // else if (gamepad1.y){
            //     slide.setPower(-0.6);
            // }
            // else {
            //     slide.setPower(0);
            // }



            // toggle claw state
            if (gamepad1.a && !aPreviouslyPressed) {
                clawOpen = !clawOpen;
            }
            aPreviouslyPressed = gamepad1.a;


            // move claw
            if (clawOpen) {
                clawL.setPosition(clawLOpen);
                clawR.setPosition(clawROpen);
            } else if (!clawOpen) {
                clawL.setPosition(clawLClose);
                clawR.setPosition(clawRClose);
            }


            // // toggle wrist state
            // if (gamepad1.b && !bPreviouslyPressed) {
            //     wristParallel = !wristParallel;
            // }
            // bPreviouslyPressed = gamepad1.b;


            // // move wrist
            // if (wristParallel) {
            //     wristL.setPosition(wristLOuttake2);
            //     wristR.setPosition(wristROuttake2);
            // } else if (!wristParallel) {
            //     wristL.setPosition(wristLOuttake);
            //     wristR.setPosition(wristROuttake);
            // }


            // arm
            arm.setPower(armPIDF(armTarget, arm));
            slide.setPower(slidePID(slideTarget, slide));

//            // arm movement with PID
//            if (gamepad1.right_trigger > 0 && lSlide.getCurrentPosition() < 720) {
//                telemetry.addData("dpad_up", "active");
//                telemetry.update();
//                lSlide.setPower(0.4);
//                rSlide.setPower(0.4);
//            } else if (gamepad1.left_trigger > 0 && lSlide.getCurrentPosition() > 0) {
//                lSlide.setPower(-0.25);
//                rSlide.setPower(-0.25);
//
//            } else {
//                if (gamepad1.right_trigger > 0) { // Prevent jittering
//                    lSlide.setPower(0.15);
//                    rSlide.setPower(0.15);
//                } else {
//                    lSlide.setPower(0);
//                    rSlide.setPower(0);
//                }
//            }


            // automatic 30deg claw
            currArmPose = arm.getCurrentPosition();
            if (currArmPose <= armUpperLimit-1) {
                adjustmentFactor = (currArmPose - armUpperLimit) * (adjustmentMultiplier);
                wristL.setPosition(0+adjustmentFactor);
            }
            // add limit



            telemetry.addData("Arm pose" , arm.getCurrentPosition());
            // telemetry.addData("WristL pose", 0+adjustmentFactor);
            // telemetry.addData("WristR pose", 0.96-adjustmentFactor);
            telemetry.addData("adjustmentMultiplier", adjustmentMultiplier);
            telemetry.addData("part1", currArmPose - armUpperLimit);
            telemetry.addData("adjustmentFactor", adjustmentFactor);



            telemetry.update();
        }
    }

    public double armPIDF(int target, DcMotor motor){
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        telemetry.addData("arm current position: ", currentPosition);
        telemetry.addData("arm target: ", target);
        telemetry.update();
        return output;
    }
    public double slidePID(int target, DcMotor motor){
        slidePID.setPID(slideP, slideI, slideD);
        int currentPosition = motor.getCurrentPosition();
        double output = slidePID.calculate(currentPosition, target);

        telemetry.addData("slide current position: ", currentPosition);
        telemetry.addData("slide target: ", target);
        telemetry.update();
        return output;
    }
}





//package org.firstinspires.ftc.teamcode.drive.teleop;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//
//@Config
//@TeleOp(name="MecanumTeleop with PID", group="Mecanum")
//public class MecanumTeleop extends LinearOpMode {
//
//    // Declare servos and motors
//    public Servo wristR = null;
//    public Servo armL = null;
//    public Servo armR = null;
//    public Servo wristL = null;
//    public Servo clawR = null;
//    public Servo clawL = null;
//    public Servo hookR = null;
//    public Servo hookL = null;
//    public Servo droneLauncher = null;
//    public DcMotor fr, fl, br, bl, lSlide, rSlide, lClimb, rClimb;
//
//    boolean clawOpen = false;
//    boolean clawLOpen = false;
//    boolean clawROpen = false;
//    boolean isOuttaking = false;
//
//    double lIntake = 0.77;
//    double rIntake = 0.29;
//    double lOuttake = 0.16;
//    double rOuttake = 0.90;
//
//    //Slides pid
//    public static PIDController pid = new PIDController(0,0,0);
//
//    public static double p = 0.04, i = 0, d = 0.00015, f = 0;
//
//    public static int target = 0;
//
//
//
//
//
//    // Initialize standard Hardware interfaces
//    public void initHardware() {
//        //Tuning PID
//
//
//        // Servos
//        wristR = hardwareMap.get(Servo.class, "wristR");
//        armL = hardwareMap.get(Servo.class, "armL");
//        armR = hardwareMap.get(Servo.class, "armR");
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        clawR = hardwareMap.get(Servo.class, "clawR");
//        clawL = hardwareMap.get(Servo.class, "clawL");
//        hookR = hardwareMap.get(Servo.class, "hookR");
//        hookL = hardwareMap.get(Servo.class, "hookL");
//        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
//
//        //--------------------------------------------------MAYBE IMPORTANT---------------------
//        // armL.setPosition(1); //towards 0 is clockwise
//        // armR.setPosition(0); //towards 1 is clockwise +0.25 = armL
//        // wristR.setPosition(0.85);
//        // wristL.setPosition(0.85); //both the same 0 is counterclockwise
//        // clawR.setPosition(0.47);
//        // clawL.setPosition(0.62);
//
//
//
//        // Motors
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        lSlide = hardwareMap.get(DcMotor.class, "lSlide");
//        rSlide = hardwareMap.get(DcMotor.class, "rSlide");
//        lClimb = hardwareMap.get(DcMotor.class, "lClimb");
//        rClimb = hardwareMap.get(DcMotor.class, "rClimb");
//
//        fr.setPower(0);
//        fl.setPower(0);
//        br.setPower(0);
//        bl.setPower(0);
//        lSlide.setPower(0);
//        rSlide.setPower(0);
//        lClimb.setPower(0);
//        rClimb.setPower(0);
//
//
//        fr.setDirection(DcMotor.Direction.REVERSE);
//        br.setDirection(DcMotor.Direction.REVERSE);
//        fl.setDirection(DcMotor.Direction.FORWARD);
//        bl.setDirection(DcMotor.Direction.FORWARD);
//        lSlide.setDirection(DcMotor.Direction.REVERSE);
//        rSlide.setDirection(DcMotor.Direction.FORWARD);
//        lClimb.setDirection(DcMotor.Direction.FORWARD);
//        rClimb.setDirection(DcMotor.Direction.FORWARD);
//
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        lClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lClimb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rClimb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    @Override
//    public void runOpMode() {
//        initHardware();
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("armL port number", armR.getPortNumber());
//
//        telemetry.update();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//        waitForStart();
//
//        wristR.setPosition(0.93);
//        wristL.setPosition(0.93);
//        armL.setPosition(0);
//        armR.setPosition(0);
//
//        while (opModeIsActive()) {
//
//            telemetry.addData("Status", "Running");
////            lSlide.setPower(slidePID(target, lSlide));
////            rSlide.setPower(slidePID(target, lSlide));
//
//            // Motors: driving
//            double drive = -gamepad1.left_stick_y;
//            double strafe = gamepad1.left_stick_x;
//            double rotate = gamepad1.right_stick_x;
//            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
//            double frontLeftPower = (drive + strafe + rotate) / denominator * 1.4;
//            double backLeftPower = (drive - strafe + rotate) / denominator * 1.4;
//            double frontRightPower = (drive - strafe - rotate) / denominator * 1.4;
//            double backRightPower = (drive + strafe - rotate) / denominator * 1.4;
//            fr.setPower(frontRightPower);
//            fl.setPower(frontLeftPower);
//            br.setPower(backRightPower);
//            bl.setPower(backLeftPower);
//            telemetry.addData("strafe", "%.2f", strafe);
//            telemetry.addData("drive", "%.2f", drive);
//            telemetry.addData("rotate", "%.2f", rotate);
//            telemetry.addData("lSlide Pos", lSlide.getCurrentPosition());
//            telemetry.addData("rSlide Pos", rSlide.getCurrentPosition());
//
//
//            // Both claw control
//            if (gamepad1.a && (!clawOpen)) {//open claw
//                if (!isOuttaking) {
//                    wristR.setPosition(0.58);
//                    wristL.setPosition(0.58); // 0.6
//                    armL.setPosition(0);
//                    armR.setPosition(0);
//                    sleep(100);
//                }
//                clawR.setPosition(0.6);
//                clawL.setPosition(0.6);
////                sleep(100);
//                clawOpen = true;
//                clawLOpen = true;
//                clawROpen = true;
//            } else if (gamepad1.a && (clawOpen)) { //close claw
//                clawR.setPosition(0.33);
//                clawL.setPosition(0.33);
//                sleep(100);
//                if (!isOuttaking) {
//                    telemetry.addData("claw ", "isOuttaking");
//                    wristR.setPosition(0.9);
//                    wristL.setPosition(0.9);
//                    armL.setPosition(0.1);
//                    armR.setPosition(0.1);
//                }
//                clawOpen = false;
//                clawLOpen = false;
//                clawROpen = false;
//            }
//
//            // Left claw control
//            if (gamepad1.x && clawLOpen) { // opening left claw
//                clawL.setPosition(0.6);
////                sleep(500);
//                clawLOpen = false;
//            } else if (gamepad1.x && !clawLOpen) {
//                clawL.setPosition(0.33);
////                sleep(500);
//                clawLOpen = true;
//            }
//
//            // Right claw control
//            if (gamepad1.b && clawROpen) { // opening right claw
//                clawR.setPosition(0.6);
////                sleep(500);
//                clawROpen = false;
//            } else if (gamepad1.b && !clawROpen) {
//                clawR.setPosition(0.33);
////                sleep(500);
//                clawROpen = true;
//            }
//
//
//            //OUTTAKING
//            if (gamepad1.y && (!isOuttaking)) { // going to Outtaking position
//                wristR.setPosition(0.5);
//                wristL.setPosition(0.5);
//                sleep(200);
//                armL.setPosition(0.8);
//                armR.setPosition(0.8);
////                sleep(300);
//                isOuttaking = true;
//            }
//            if (gamepad1.y && (isOuttaking)) { //going to DRIVING position
//                if (clawOpen) { // close claw
//                    clawR.setPosition(0.33);
//                    clawL.setPosition(0.33);
////                    sleep(100);
//                    clawOpen = false;
//                    clawLOpen = false;
//                    clawROpen = false;
//                }
//                wristR.setPosition(0.5);
//                wristL.setPosition(0.5);
//                sleep(100);
//                armL.setPosition(0.1);
//                armR.setPosition(0.1);
////                sleep(300);
//                wristR.setPosition(0.9);
//                wristL.setPosition(0.9);
//                isOuttaking = false;
//            }
//            telemetry.addData("isOuttaking ", isOuttaking);
//            telemetry.addData("lSlidepose ", lSlide.getCurrentPosition());
//            if (isOuttaking && lSlide.getCurrentPosition() > 230) { // flip after reaches certian height
//                armL.setPosition(0.9);
//                armR.setPosition(0.9);
//                wristR.setPosition(0.9);
//                wristL.setPosition(0.9);
//            }
//
//            // Slides
//            if (gamepad1.right_trigger > 0 && lSlide.getCurrentPosition() < 720) { // up
//                telemetry.addData("dpad_up","active");
//                telemetry.update();
//                lSlide.setPower(0.4);
//                rSlide.setPower(0.4);
//            } else if (gamepad1.left_trigger >0 &&lSlide.getCurrentPosition() > 0 ) { // down
//                lSlide.setPower(-0.25);
//                rSlide.setPower(-0.25);
//            } else {
//                if (gamepad1.right_trigger > 0) { // Prevent jittering
//                    lSlide.setPower(0.15);
//                    rSlide.setPower(0.15);
//                } else {
//                    lSlide.setPower(0);
//                    rSlide.setPower(0);
//                }
//            }
//
//            if (gamepad1.dpad_up) {
//                droneLauncher.setPosition(0.7);
//            }
//
//
//
//             // Slides PID
////            telemetry.addData("target ", target);
////            if (gamepad1.right_trigger > 0 && target < 720){
////                target += 9;
////
////            } else if (gamepad1.left_trigger > 0 && target > 12) {
////                if ( target > 648) {
////                    target -= 3;
////                } else {
////                    target -= 12;
////                }
////            }
//
//            // Climb
////            if (gamepad1.dpad_left) {
////                lClimb.setPower(0.7);
////                rClimb.setPower(0.7);
////            } else if (gamepad1.dpad_right) {
////                lClimb.setPower(-0.7);
////                rClimb.setPower(-0.7);
////            } else {
////                lClimb.setPower(0);
////                rClimb.setPower(0);
////            }
////            while (gamepad1.dpad_up) {
////                hookL.setPosition(1);
////                hookR.setPosition(1);
////            }
////                hookL.setPosition(0);
////                hookR.setPosition(0);
//
//            telemetry.update();
//        }
//    }
//
//    public double slidePID(int target, DcMotor motor){
//        pid.setPID(p,i,d);
//        int currentPosition = motor.getCurrentPosition();
//        double output = pid.calculate(currentPosition, target);
//
//        telemetry.addData("current position: ", currentPosition);
//        telemetry.addData("target: ", target);
////        telemetry.update();
//        return output;
//    }
//}
//
//
//
//
//
//
