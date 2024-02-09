package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonRoadrunner: red right middle", group = "auton")
public class AutonRoadrunner extends OpMode {

//    public Servo clawL = null, clawR = null, wristL = null, wristR = null;
//    public DcMotor arm, slide;
//
//    double clawLOpen = 0;
//    double clawROpen = 1;
//    double clawLClose = 0.6;
//    double clawRClose = 0.4;
//
//    double wristLParallel = 0.96-0.052;
//    double wristRParallel = 0+0.052;
//    double wristLUp = 0+0.35;
//    double wristRUp = 0.96-0.35;
//
//    double armUpperLimit = 1534;
//    double armLowerLimit = 1773;
//    double adjustmentMultiplier = 0.17/239; // 0.17/239
//    double adjustmentFactor = 0;
//
//    double armOuttakingTarget = 1600;
//    double slideExtendedTarget = 400;
//    double armIntakingTarget = 150;
//    double slideRetractedTarget = 40;
//
//
//    // Arm pidf
//    public static PIDFController armPIDF = new PIDFController(0,0,0,0);
//    public static double armP = 0.029, armI = 0, armD = 0.002, armF = 0.005;
//    public static double armTarget = 0.0; // limit 1900, 70
//    // Slides pid
//    public static PIDController slidePID = new PIDController(0,0,0);
//    public static double slideP = 0.3, slideI = 0, slideD = 0.005;
//    public static double slideTarget = 0.0; // limit 670


    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Trajectory toSpike, toBoardFromSpike, toStack, toBoardFromStack, toParking;
    Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));

    public void init() {
        // SERVOS
//        clawL = hardwareMap.get(Servo.class, "clawL");
//        clawR = hardwareMap.get(Servo.class, "clawR");
//        clawL.setPosition(clawLClose);
//        clawR.setPosition(clawRClose);
//
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        wristR = hardwareMap.get(Servo.class, "wristR");
//        wristL.setPosition(wristLUp);
//        wristR.setPosition(wristRUp);
//
//        // MOTORS
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        slide = hardwareMap.get(DcMotor.class, "slide");
//
//        arm.setDirection(DcMotor.Direction.REVERSE);
//        slide.setDirection(DcMotor.Direction.FORWARD);
//
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        slide.setPower(0);
//        arm.setPower(0);

        // TRAJECTORIES
        toSpike = drive.trajectoryBuilder((startPose))
                .splineToLinearHeading(new Pose2d(16, -36, Math.toRadians(90)), Math.toRadians(0))

//                .addTemporalMarker(0.5, 0, () -> {
//                    wristL.setPosition(wristLParallel);
//                    wristR.setPosition(wristRParallel);
//                })
//
//                .addTemporalMarker(1, -0.5, () -> {
//                    clawL.setPosition(clawLOpen);
//                })

                .addTemporalMarker(1, 1,() -> {
//                    wristL.setPosition(wristLUp);
//                    wristR.setPosition(wristRUp);
                    drive.followTrajectoryAsync(toBoardFromSpike);
                })
                .build();

        toBoardFromSpike = drive.trajectoryBuilder((toSpike.end()))
                .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(90))

//                .addTemporalMarker(0, 0, () -> {
//                    armTarget = armOuttakingTarget;
//                    slideTarget = slideExtendedTarget;
//                    autoWrist();
//                })
//                .addTemporalMarker(1, -0.5, () -> {
//                    armTarget = armOuttakingTarget - 100;
//                })

                .addTemporalMarker(1, 0, () -> {
//                    clawR.setPosition(clawROpen);
                    drive.followTrajectoryAsync(toStack);
                })
                .build();

        toStack = drive.trajectoryBuilder((toBoardFromSpike.end()))
                .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

//                .addDisplacementMarker(2, () -> {
//                    slideTarget = slideRetractedTarget;
//                    armTarget = armIntakingTarget;
//                    wristL.setPosition(wristLParallel);
//                    wristR.setPosition(wristRParallel);
//                    clawR.setPosition(clawRClose);
//                })
//                .addTemporalMarker(1, -0.5, () -> {
//                    slideTarget = slideExtendedTarget;
//                    armTarget = armIntakingTarget + 20;
//                })
//                .addTemporalMarker(1, 0.5, () -> {
//                    armTarget = armIntakingTarget;
//                    clawL.setPosition(clawLClose);
//                })
                .addTemporalMarker(1, 1, () -> drive.followTrajectoryAsync(toBoardFromStack))
                .build();

        toBoardFromStack = drive.trajectoryBuilder((toStack.end()), true)
                .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

//                .addTemporalMarker(0, 0, () -> {
//                    slideTarget = slideRetractedTarget;
//                    wristL.setPosition(wristLUp);
//                    wristR.setPosition(wristRUp);
//                })
//                .addTemporalMarker(1, -0.5, () -> {
//                    slideTarget = slideExtendedTarget;
//                    armTarget = armOuttakingTarget;
//                    autoWrist();
//
//                })
//                .addTemporalMarker(1, 0, () -> {
//                    armTarget = armOuttakingTarget - 100;
//                })
                .addTemporalMarker(1, 0.5, () -> {
//                    clawL.setPosition(clawLOpen);
                    drive.followTrajectoryAsync(toParking);
                })
                .build();

        toParking = drive.trajectoryBuilder((toBoardFromStack.end()))
                .splineToLinearHeading(new Pose2d(52, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(56, -59, Math.toRadians(180)), Math.toRadians(0))
//                .addTemporalMarker(0, 0.2, () -> {
//                    slideTarget = slideRetractedTarget;
//                    armTarget = armIntakingTarget;
//                    wristL.setPosition(wristLUp);
//                    wristR.setPosition(wristRUp);
//                    clawL.setPosition(clawLClose);
//                })
                .build();

        drive.followTrajectoryAsync(toSpike);

    }

    public void loop() {
        drive.update();

//        slide.setPower(slidePID(slideTarget, slide));
//        arm.setPower(armPIDF(armTarget, arm));
//        telemetry.addData("slide curr pose:", slide.getCurrentPosition());
//        telemetry.addData("arm curr pose:", arm.getCurrentPosition());
//        telemetry.update();

    }

//    public double armPIDF(double target, DcMotor motor){
//        armPIDF.setPIDF(armP,armI,armD,armF);
//        int currentPosition = motor.getCurrentPosition();
//        double output = armPIDF.calculate(currentPosition, target);
//
////        telemetry.addData("arm current position: ", currentPosition);
////        telemetry.addData("arm target: ", target);
////        telemetry.update();
//        return output/5;
//    }
//    public double slidePID(double target, DcMotor motor){
//        slidePID.setPID(slideP, slideI, slideD);
//        int currentPosition = motor.getCurrentPosition();
//        double output = slidePID.calculate(currentPosition, target);
//
////        telemetry.addData("slide current position: ", currentPosition);
////        telemetry.addData("slide target: ", target);
////        telemetry.update();
//        return output/5;
//    }
//    public void autoWrist() {
//        if (slideTarget > 100) { // extended enough for claw
//            double currArmPose = arm.getCurrentPosition(); // change back once arm is fixed
//            if (currArmPose > armUpperLimit) {
//                adjustmentFactor = (currArmPose - armUpperLimit) * (adjustmentMultiplier);
//                wristL.setPosition(0+adjustmentFactor);
//                wristR.setPosition(0.96-adjustmentFactor);
//            }
//            telemetry.addData("adjustmentFactor", adjustmentFactor);
//        }
//        else {
//            // Perpendicular
//            wristL.setPosition(wristLUp);
//            wristR.setPosition(wristRUp);
//        }
//        return;
//    }
}



//package org.firstinspires.ftc.teamcode.drive.auton;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@Autonomous(name = "AutonRoadrunner: red right middle", group = "auton")
//public class AutonRoadrunner extends OpMode {
//
//    public Servo clawL = null, clawR = null, wristL = null, wristR = null;
//    public DcMotor arm, slide;
//
//    double clawLOpen = 0;
//    double clawROpen = 1;
//    double clawLClose = 0.6;
//    double clawRClose = 0.4;
//
//    double wristLParallel = 0.96-0.052;
//    double wristRParallel = 0+0.052;
//    double wristLUp = 0+0.35;
//    double wristRUp = 0.96-0.35;
//
//    double armUpperLimit = 1534;
//    double armLowerLimit = 1773;
//    double adjustmentMultiplier = 0.17/239; // 0.17/239
//    double adjustmentFactor = 0;
//
//    double armOuttakingTarget = 1600;
//    double slideExtendedTarget = 400;
//    double armIntakingTarget = 150;
//    double slideRetractedTarget = 40;
//
//
//    // Arm pidf
//    public static PIDFController armPIDF = new PIDFController(0,0,0,0);
//    public static double armP = 0.029, armI = 0, armD = 0.002, armF = 0.005;
//    public static double armTarget = 0.0; // limit 1900, 70
//    // Slides pid
//    public static PIDController slidePID = new PIDController(0,0,0);
//    public static double slideP = 0.3, slideI = 0, slideD = 0.005;
//    public static double slideTarget = 0.0; // limit 670
//
//
//    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//    Trajectory toSpike, toBoardFromSpike, toStack, toBoardFromStack, toParking;
//    Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));
//
//    public void init() {
//        // SERVOS
//        clawL = hardwareMap.get(Servo.class, "clawL");
//        clawR = hardwareMap.get(Servo.class, "clawR");
//        clawL.setPosition(clawLClose);
//        clawR.setPosition(clawRClose);
//
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        wristR = hardwareMap.get(Servo.class, "wristR");
//        wristL.setPosition(wristLUp);
//        wristR.setPosition(wristRUp);
//
//        // MOTORS
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        slide = hardwareMap.get(DcMotor.class, "slide");
//
//        arm.setDirection(DcMotor.Direction.REVERSE);
//        slide.setDirection(DcMotor.Direction.FORWARD);
//
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        slide.setPower(0);
//        arm.setPower(0);
//
//        // TRAJECTORIES
//        toSpike = drive.trajectoryBuilder((startPose))
//                .splineToLinearHeading(new Pose2d(16, -36, Math.toRadians(90)), Math.toRadians(0))
//
//                .addTemporalMarker(0.5, 0, () -> {
//                    wristL.setPosition(wristLParallel);
//                    wristR.setPosition(wristRParallel);
//                })
//
//                .addTemporalMarker(1, -0.5, () -> {
//                    clawL.setPosition(clawLOpen);
//                })
//
//                .addTemporalMarker(1, 1,() -> {
//                    wristL.setPosition(wristLUp);
//                    wristR.setPosition(wristRUp);
//                    drive.followTrajectoryAsync(toBoardFromSpike);
//                })
//                .build();
//
//        toBoardFromSpike = drive.trajectoryBuilder((toSpike.end()))
//                .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(90))
//
//                .addTemporalMarker(0, 0, () -> {
//                    armTarget = armOuttakingTarget;
//                    slideTarget = slideExtendedTarget;
//                    autoWrist();
//                })
//                .addTemporalMarker(1, -0.5, () -> {
//                    armTarget = armOuttakingTarget - 100;
//                })
//
//                .addTemporalMarker(1, 0, () -> {
//                    clawR.setPosition(clawROpen);
//                    drive.followTrajectoryAsync(toStack);
//                })
//                .build();
//
//        toStack = drive.trajectoryBuilder((toBoardFromSpike.end()))
//                .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))
//
//                .addDisplacementMarker(2, () -> {
//                    slideTarget = slideRetractedTarget;
//                    armTarget = armIntakingTarget;
//                    wristL.setPosition(wristLParallel);
//                    wristR.setPosition(wristRParallel);
//                    clawR.setPosition(clawRClose);
//                })
//                .addTemporalMarker(1, -0.5, () -> {
//                    slideTarget = slideExtendedTarget;
//                    armTarget = armIntakingTarget + 20;
//                })
//                .addTemporalMarker(1, 0.5, () -> {
//                    armTarget = armIntakingTarget;
//                    clawL.setPosition(clawLClose);
//                })
//                .addTemporalMarker(1, 1, () -> drive.followTrajectoryAsync(toBoardFromStack))
//                .build();
//
//        toBoardFromStack = drive.trajectoryBuilder((toStack.end()), true)
//                .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))
//
//                .addTemporalMarker(0, 0, () -> {
//                    slideTarget = slideRetractedTarget;
//                    wristL.setPosition(wristLUp);
//                    wristR.setPosition(wristRUp);
//                })
//                .addTemporalMarker(1, -0.5, () -> {
//                    slideTarget = slideExtendedTarget;
//                    armTarget = armOuttakingTarget;
//                    autoWrist();
//
//                })
//                .addTemporalMarker(1, 0, () -> {
//                    armTarget = armOuttakingTarget - 100;
//                })
//                .addTemporalMarker(1, 0.5, () -> {
//                    clawL.setPosition(clawLOpen);
//                    drive.followTrajectoryAsync(toParking);
//                })
//                .build();
//
//        toParking = drive.trajectoryBuilder((toBoardFromStack.end()))
//                .splineToLinearHeading(new Pose2d(52, -59, Math.toRadians(180)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(56, -59, Math.toRadians(180)), Math.toRadians(0))
//                .addTemporalMarker(0, 0.2, () -> {
//                    slideTarget = slideRetractedTarget;
//                    armTarget = armIntakingTarget;
//                    wristL.setPosition(wristLUp);
//                    wristR.setPosition(wristRUp);
//                    clawL.setPosition(clawLClose);
//                })
//                .build();
//
//        drive.followTrajectoryAsync(toSpike);
//
//    }
//
//    public void loop() {
//        drive.update();
//
//        slide.setPower(slidePID(slideTarget, slide));
//        arm.setPower(armPIDF(armTarget, arm));
//        telemetry.addData("slide curr pose:", slide.getCurrentPosition());
//        telemetry.addData("arm curr pose:", arm.getCurrentPosition());
//        telemetry.update();
//
//    }
//
//    public double armPIDF(double target, DcMotor motor){
//        armPIDF.setPIDF(armP,armI,armD,armF);
//        int currentPosition = motor.getCurrentPosition();
//        double output = armPIDF.calculate(currentPosition, target);
//
////        telemetry.addData("arm current position: ", currentPosition);
////        telemetry.addData("arm target: ", target);
////        telemetry.update();
//        return output/5;
//    }
//    public double slidePID(double target, DcMotor motor){
//        slidePID.setPID(slideP, slideI, slideD);
//        int currentPosition = motor.getCurrentPosition();
//        double output = slidePID.calculate(currentPosition, target);
//
////        telemetry.addData("slide current position: ", currentPosition);
////        telemetry.addData("slide target: ", target);
////        telemetry.update();
//        return output/5;
//    }
//    public void autoWrist() {
//        if (slideTarget > 100) { // extended enough for claw
//            double currArmPose = arm.getCurrentPosition(); // change back once arm is fixed
//            if (currArmPose > armUpperLimit) {
//                adjustmentFactor = (currArmPose - armUpperLimit) * (adjustmentMultiplier);
//                wristL.setPosition(0+adjustmentFactor);
//                wristR.setPosition(0.96-adjustmentFactor);
//            }
//            telemetry.addData("adjustmentFactor", adjustmentFactor);
//        }
//        else {
//            // Perpendicular
//            wristL.setPosition(wristLUp);
//            wristR.setPosition(wristRUp);
//        }
//        return;
//    }
//}
