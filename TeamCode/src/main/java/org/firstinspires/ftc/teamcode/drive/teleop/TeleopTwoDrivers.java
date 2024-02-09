package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
@TeleOp(name="MecanumTeleopTwoDrivers", group="Mecanum")
public class TeleopTwoDrivers extends LinearOpMode {

    private ElapsedTime delay = new ElapsedTime();

    // Declare servos and motors
    public Servo clawL = null, clawR = null, wristL = null, wristR = null, droneAngle = null, droneRelease = null;
    public DcMotor arm, slide, fl, fr, bl, br;

    double droneAngleRest = 0;
    double droneAngleActive = 0.4;
    double droneReleaseRest = 0.3;
    double droneReleaseActive = 0.5;

    double clawLOpen = 0;
    double clawROpen = 1;
    double clawLClose = 0.6;
    double clawRClose = 0.4;

    double armUpperLimit = 1420;
    double armLowerLimit = 1740;
    double currArmPose = 0;
    double adjustmentMultiplier = 0.18/239;
    double adjustmentFactor = 0;
    double wristLParallel = 0.96-0.052;
    double wristRParallel = 0+0.052;
    double wristLUp = 0+0.35;
    double wristRUp = 0.96-0.35;
    double armRestPose = 100;

    double tempArmTarget = 0.0;
    double tempSlideTarget = 0.0;
    double slidePowerAdjustment = 0.7;
    boolean droneAimed = false;

    boolean RBNotPressed = true;

    public enum Mode {
        REST,
        INTAKING,
        OUTTAKING
    }
    Mode mode = Mode.REST;

    // Arm pidf
    public static PIDFController armPIDF = new PIDFController(0,0,0,0);
    public static double armP = 0.029, armI = 0, armD = 0.002, armF = 0.005;
    public static double armTarget = 0.0; // limit 1900, 70
    // Slides pid
    public static PIDController slidePID = new PIDController(0,0,0);
    public static double slideP = 0.3, slideI = 0, slideD = 0.005;
    public static double slideTarget = 0.0; // limit 670


    // Initialize standard Hardware interfaces
    public void initHardware() {
        // Servos
        droneAngle = hardwareMap.get(Servo.class, "droneAngle");
        droneRelease = hardwareMap.get(Servo.class, "droneRelease");
        droneAngle.setPosition(droneAngleRest);
        droneRelease.setPosition(droneReleaseRest);

        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        clawL.setPosition(clawLClose);
        clawR.setPosition(clawRClose);

        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        // initialize vertically
        wristL.setPosition(wristLUp);
        wristR.setPosition(wristRUp);

        // Motors
        arm = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

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

        slide.setPower(0);
        arm.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

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

            // Motors
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);

            double frontLeftPower = (drive + strafe + rotate) / denominator * 1.2;
            double backLeftPower = (drive - strafe + rotate) / denominator * 1.2;
            double frontRightPower = (drive - strafe - rotate) / denominator * 1.2;
            double backRightPower = (drive + strafe - rotate) / denominator * 1.2;

            fr.setPower(frontRightPower);
            fl.setPower(frontLeftPower);
            br.setPower(backRightPower);
            bl.setPower(backLeftPower);

            // Drone
            if (gamepad1.x && !droneAimed) {
                droneAngle.setPosition(droneAngleActive);
            } else if (gamepad1.x && droneAimed) {
                droneRelease.setPosition(droneReleaseActive);
            }

            // Arm and Slide PID
            if (slideTarget < 740 && slideTarget >= 40) {
                slide.setPower(slidePID(slideTarget, slide));
            }
            if (armTarget < 1800 && armTarget >= 0) {
                arm.setPower(armPIDF(armTarget, arm));
            }
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("slideTarget", slideTarget);

            telemetry.addData("mode", mode);
            telemetry.addData("delay", delay);
            // Mode: REST, INTAKING, OUTTAKING
            switch (mode) {
                case REST:
                    telemetry.addData("mode", "rest");
                    telemetry.update();
                    RBNotPressed = true;
                    armRestPose -= (gamepad2.left_trigger - gamepad2.right_trigger) * 2;
                    armTarget = armRestPose;
                    slideTarget = 40;

                    wristL.setPosition(wristLUp);
                    wristR.setPosition(wristRUp);
                    clawL.setPosition(clawLClose);
                    clawR.setPosition(clawRClose);

                    if (gamepad1.right_bumper || gamepad1.left_bumper) {
                        slidePowerAdjustment = 0.7;
                        mode = Mode.INTAKING;
                        delay.reset();
                    } else if (gamepad2.right_bumper && gamepad2.left_bumper && delay.seconds() >= 0.5) {
                        slidePowerAdjustment = 0.7;
                        mode = Mode.OUTTAKING;
                        armTarget = armUpperLimit - 300;
                        delay.reset();
                    }
                    break;

                case INTAKING:
                    telemetry.addData("mode", "intaking");
                    telemetry.update();

                    telemetry.addData("controlledArmTarget",  (gamepad2.left_trigger - gamepad2.right_trigger) * 2);
//                    telemetry.addData("controlledSlideTarget",  gamepad2.left_stick_y * 2);
                    armRestPose -= (gamepad2.left_trigger - gamepad2.right_trigger) * 2;
                    armTarget = armRestPose;
//                    slideTarget = slideTarget - gamepad2.left_stick_y * 2;

                    if (gamepad1.right_bumper || gamepad1.left_bumper) {
                        if (delay.seconds() > 0.2 && delay.seconds() < 0.3) {
                            armTarget = 40;
                        }
                        if (gamepad1.right_bumper) {
                            slideTarget = 620;
                        } else if (gamepad1.left_bumper) {
                            slideTarget = 400;
                        }
                        clawL.setPosition(clawLOpen);
                        clawR.setPosition(clawROpen);
                        wristL.setPosition(wristLParallel);
                        wristR.setPosition(wristRParallel);
                    }
                    if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                        if (RBNotPressed) {
                            slidePowerAdjustment = 0.1;
                            RBNotPressed = false;
                            delay.reset();
                            clawL.setPosition(clawLClose);
                            clawR.setPosition(clawRClose);
                        }
                        else if (delay.seconds() >= 0.8) {
                            telemetry.addData("changing mode", "true");
                            mode = Mode.REST;
                        }
                    }
                    break;

                case OUTTAKING:
                    telemetry.addData("mode", "outtaking");
                    telemetry.update();
                    if (delay.seconds() > 0.9 && delay.seconds() < 1) {
                        slideTarget = 250;
                    }

                    telemetry.addData("controlledArmTarget",  (gamepad2.left_trigger - gamepad2.right_trigger) * 5);
                    telemetry.addData("controlledSlideTarget",  gamepad2.left_stick_y * 5);
                    armTarget = armTarget + (gamepad2.left_trigger - gamepad2.right_trigger) * 5;
                    slideTarget = slideTarget - gamepad2.left_stick_y * 5;

                    // claw
                    if (gamepad2.a) {
                        clawL.setPosition(clawLOpen);
                        clawR.setPosition(clawROpen);
                    }

                    // auto wrist
                    if (delay.seconds() > 1.1) {
                        if (slideTarget > 200) { // extended enough for claw
                            currArmPose = arm.getCurrentPosition(); // change back once arm is fixed
                            if (currArmPose > armUpperLimit) {
                                adjustmentFactor = (currArmPose - armUpperLimit) * (adjustmentMultiplier);
                                wristL.setPosition(0+adjustmentFactor);
                                wristR.setPosition(0.96-adjustmentFactor);
                            }
                            telemetry.addData("adjustmentFactor", adjustmentFactor);
                        }
                        else {
                            // Perpendicular
                            wristL.setPosition(0+0.35);
                            wristR.setPosition(0.96-0.35);
                        }
                    }

                    if (gamepad2.right_bumper && gamepad2.left_bumper && delay.seconds() >= 0.5) { // prevent double click
                        slidePowerAdjustment = 0.1;
                        delay.reset();
                        RBNotPressed = false;
                        slideTarget = 40;
                        clawL.setPosition(clawLClose);
                        clawR.setPosition(clawRClose);
                        wristL.setPosition(wristLUp);
                        wristR.setPosition(wristRUp);
                    }
                    if (delay.seconds() >= 0.6 && !RBNotPressed) {
                        mode = Mode.REST;
                        delay.reset();
                    }
                    break;
            }
        }
        telemetry.update();
    }

    public double armPIDF(double target, DcMotor motor){
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

//        telemetry.addData("arm current position: ", currentPosition);
//        telemetry.addData("arm target: ", target);
//        telemetry.update();
        return output/5;
    }
    public double slidePID(double target, DcMotor motor){
        slidePID.setPID(slideP, slideI, slideD);
        int currentPosition = motor.getCurrentPosition();
        double output = slidePID.calculate(currentPosition, target);

//        telemetry.addData("slide current position: ", currentPosition);
//        telemetry.addData("slide target: ", target);
//        telemetry.update();
        return output/5;
    }
}



// Single driver

//package org.firstinspires.ftc.teamcode.drive.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//
//@Config
//@TeleOp(name="MecanumTeleop", group="Mecanum")
//public class MecanumTeleop extends LinearOpMode {
//
//    // Declare servos and motors
//    public Servo clawL = null, clawR = null, wristL = null, wristR = null;
//    public DcMotor arm, slide, fl, fr, bl, br;
//
//    double clawLOpen = 0.1;
//    double clawROpen = 0.9;
//    double clawLClose = 0.44;
//    double clawRClose = 0.60;
//
//    double armUpperLimit = -1534;
//    double armLowerLimit = -1773;
//    double currArmPose = 0;
//    double adjustmentMultiplier = 0.17/-239;
//    double adjustmentFactor = 0;
//    double wristLParallel = 0.96-0.05;
//    double wristRParallel = 0+0.05;
//    boolean modeChangePressed = false;
//
//    boolean intakeActive = false;
//    int intakeMode = 1;
//    double tempArmTarget = 0.0;
//    double tempSlideTarget = 0.0;
//
//    // Arm pidf
//    public static PIDFController armPIDF = new PIDFController(0,0,0,0);
//    public static double armP = 0.012, armI = 0, armD = 0, armF = 0.00015;
//    public static double armTarget = 0.0; // limit 1900, 70
//    // Slides pid
//    public static PIDController slidePID = new PIDController(0,0,0);
//    public static double slideP = 0.045, slideI = 0, slideD = 0;
//    public static double slideTarget = 0.0; // limit 670
//
//
//    // Initialize standard Hardware interfaces
//    public void initHardware() {
//        // Servos
//        clawL = hardwareMap.get(Servo.class, "clawL");
//        clawR = hardwareMap.get(Servo.class, "clawR");
//
//        clawL.setPosition(0.44);
//        clawR.setPosition(0.60);
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        wristR = hardwareMap.get(Servo.class, "wristR");
//
//        // initialize vertically
//        wristL.setPosition(0+0.35);
//        wristR.setPosition(0.96-0.35);
//
//        // Motors
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        slide = hardwareMap.get(DcMotor.class, "slide");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//
//        arm.setDirection(DcMotor.Direction.REVERSE);
//        slide.setDirection(DcMotor.Direction.FORWARD);
//        fl.setDirection(DcMotor.Direction.FORWARD);
//        fr.setDirection(DcMotor.Direction.REVERSE);
//        bl.setDirection(DcMotor.Direction.FORWARD);
//        br.setDirection(DcMotor.Direction.REVERSE);
//
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        slide.setPower(0);
//        arm.setPower(armPIDF(0, arm));
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
//
//    }
//
//    @Override
//    public void runOpMode() {
//        initHardware();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Status", "Running");
//
//
//            // Motors: driving
//            double drive = -gamepad1.left_stick_y;
//            double strafe = gamepad1.left_stick_x;
//            double rotate = gamepad1.right_stick_x;
//
//            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
//
//            double frontLeftPower = (drive + strafe + rotate) / denominator * 1.2;
//            double backLeftPower = (drive - strafe + rotate) / denominator * 1.2;
//            double frontRightPower = (drive - strafe - rotate) / denominator * 1.2;
//            double backRightPower = (drive + strafe - rotate) / denominator * 1.2;
//
//            fr.setPower(frontRightPower);
//            fl.setPower(frontLeftPower);
//            br.setPower(backRightPower);
//            bl.setPower(backLeftPower);
//
//            telemetry.addData("strafe","%.2f", strafe);
//            telemetry.addData("drive","%.2f", drive);
//            telemetry.addData("rotate","%.2f", rotate);
//
//
//            slide.setPower(slidePID(slideTarget, slide));
//            arm.setPower(armPIDF(armTarget, arm));
//
//
//            //-------------------------------------Going to outtake mode----------------------------------------------
//            if (gamepad1.right_bumper && gamepad1.left_bumper && !modeChangePressed) {
//                intakeMode = (intakeMode + 1) % 2;
//                if (intakeMode == 0) { // Outtaking
//                    armTarget = 1500;
//                    slideTarget = 500;
//                } else if (intakeMode == 1) { // Intaking
//                    armTarget = 30;
//                    slideTarget = 10;
//                }
//                modeChangePressed = true;
//            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper && modeChangePressed) {
//                modeChangePressed = false;
//            }
//
//            telemetry.addData("mode", intakeMode);
//
//
//            switch (intakeMode) {
//                case 0:
//                    telemetry.addData("mode", "running false");
//                    // allow slide and angle to change
//                    tempArmTarget = armTarget + (gamepad1.right_trigger - gamepad1.left_trigger) * 0.02;
//                    if (tempArmTarget > 10 && tempArmTarget < 1800) {
//                        armTarget = tempArmTarget;
//                    }
//
//                    tempSlideTarget = slideTarget + (gamepad1.right_bumper ? 0.02 : 0) - (gamepad1.left_bumper ? 0.02 : 0);
//                    if (tempSlideTarget > 10 && tempSlideTarget < 670) {
//                        slideTarget = tempSlideTarget;
//                    }
//                    telemetry.addData("armTarget", armTarget);
//                    telemetry.addData("slideTarget", slideTarget);
//
//                    // extend slide and angle arm target
//
//                    // wrist adjustment after slide extend
//                    if (slideTarget > 50) { // extended enough for claw
//                        // currArmPose = arm.getCurrentPosition();
//                        currArmPose = armTarget;
//                        if (currArmPose > armUpperLimit) {
//                            adjustmentFactor = (currArmPose - armUpperLimit) * (adjustmentMultiplier);
//                            wristL.setPosition(0+adjustmentFactor);
//                            // wristR.setPosition(0.96-adjustmentFactor);
//                        }
//                        telemetry.addData("adjustmentFactor", adjustmentFactor);
//                        telemetry.addData("wristL", wristL.getPosition());
//                        // telemetry.addData("wristR", wristR.getPosition());
//                    }
//                    else {
//                        // Perpendicular
//                        wristL.setPosition(0+0.35);
//                        wristR.setPosition(0.96-0.35);
//                    }
//
//                    // if a, open claw
//                    if (gamepad1.a) {
//                        clawL.setPosition(clawLOpen);
//                        clawR.setPosition(clawROpen);
//                    }
//                    break;
//
//                case 1:
//                    telemetry.addData("mode", "running true");
//                    if (gamepad1.right_bumper && !gamepad1.left_bumper && !intakeActive) {
//                        telemetry.addData("mode", "intake");
//                        wristL.setPosition(wristLParallel);
//                        wristR.setPosition(wristRParallel);
//                        clawL.setPosition(clawLOpen);
//                        clawR.setPosition(clawROpen);
//                        slideTarget = 400.0;
//                        intakeActive = true;
//                    } else if (!gamepad1.right_bumper && intakeActive) {
//                        telemetry.addData("mode", "rest");
//                        clawL.setPosition(clawLClose);
//                        clawR.setPosition(clawRClose);
//                        slideTarget = 10.0;
//                        intakeActive = false;
//                    }
//                    break;
//            }
//
//            telemetry.update();
//        }
//    }
//
//    public double armPIDF(double target, DcMotor motor){
//        armPIDF.setPIDF(armP,armI,armD,armF);
//        int currentPosition = motor.getCurrentPosition();
//        double output = armPIDF.calculate(currentPosition, target);
//
//        telemetry.addData("arm current position: ", currentPosition);
//        telemetry.addData("arm target: ", target);
//        telemetry.update();
//        return output;
//    }
//    public double slidePID(double target, DcMotor motor){
//        slidePID.setPID(slideP, slideI, slideD);
//        int currentPosition = motor.getCurrentPosition();
//        double output = slidePID.calculate(currentPosition, target);
//
//        telemetry.addData("slide current position: ", currentPosition);
//        telemetry.addData("slide target: ", target);
//        telemetry.update();
//        return output;
//    }
//}
//