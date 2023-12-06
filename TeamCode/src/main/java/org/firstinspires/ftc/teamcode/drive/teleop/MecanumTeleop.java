package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="hehehaha", group="Mecanum")
public class MecanumTeleop extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("armL port number", armR.getPortNumber());

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");


            // Motors: driving
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);

            double frontLeftPower = (drive + strafe + rotate) / denominator * 1.4;
            double backLeftPower = (drive - strafe + rotate) / denominator * 1.4;
            double frontRightPower = (drive - strafe - rotate) / denominator * 1.4;
            double backRightPower = (drive + strafe - rotate) / denominator * 1.4;

            fr.setPower(frontRightPower);
            fl.setPower(frontLeftPower);
            br.setPower(backRightPower);
            bl.setPower(backLeftPower);

            telemetry.addData("strafe","%.2f", strafe);
            telemetry.addData("drive","%.2f", drive);
            telemetry.addData("rotate","%.2f", rotate);

            telemetry.addData("lSlide Pos", lSlide.getCurrentPosition());
            telemetry.addData("rSlide Pos", rSlide.getCurrentPosition());

            //claw
            if(gamepad1.a && (clawOpen == false)){//open claw
                clawR.setPosition(0.6);
                clawL.setPosition(0.7);
                sleep(300);
                clawOpen = true;
            }
            else if (gamepad1.a && (clawOpen == true)){ //close claw
                clawR.setPosition(0.33);
                clawL.setPosition(0.43);
                sleep(300);
                clawOpen = false;
            }

            //picking up position
            if (gamepad1.x && (isPickingUp == false)){ //going to picking up position
                wristR.setPosition(0.6);
                wristL.setPosition(0.6);
                armL.setPosition(1);
                armR.setPosition(0);
                sleep(300);
                isPickingUp = true;
            }
            else if (gamepad1.x && (isPickingUp == true)){ //going to DRIVING position
                wristR.setPosition(0.9);
                wristL.setPosition(0.9);
                armL.setPosition(0.9);
                armR.setPosition(0.1);
                sleep(300);
                isPickingUp = false;
            }


            //OUTTAKING
            if(gamepad1.y && (isOuttaking == false)){ // going to Outtaking position

                wristR.setPosition(0.5);
                wristL.setPosition(0.5);
                armL.setPosition(0.2);
                armR.setPosition(0.8);
                sleep(2000);
                wristR.setPosition(0.9);
                wristL.setPosition(0.9);
                sleep(300);
                isOuttaking = true;
            }
            else if(gamepad1.y && (isOuttaking == true) && (clawOpen == false)){ //going to DRIVING position

                wristR.setPosition(0.5);
                wristL.setPosition(0.5);
                armL.setPosition(0.9);
                armR.setPosition(0.1);
                sleep(1000);
                wristR.setPosition(0.9);
                wristL.setPosition(0.9);

                sleep(300);
                isOuttaking = false;
            }



            // Slides
            if (gamepad1.right_trigger > 0 && lSlide.getCurrentPosition() < 720) {
                telemetry.addData("dpad_up","active");
                telemetry.update();
                lSlide.setPower(0.4);
                rSlide.setPower(0.4);
            } else if (gamepad1.left_trigger >0 &&lSlide.getCurrentPosition() > 0 ) {
                lSlide.setPower(-0.25);
                rSlide.setPower(-0.25);
            }

            else {
                if (gamepad1.right_trigger > 0) { // Prevent jittering
                    lSlide.setPower(0.15);
                    rSlide.setPower(0.15);
                } else {
                    lSlide.setPower(0);
                    rSlide.setPower(0);
                }
            }




            telemetry.update();
        }
    }
}






