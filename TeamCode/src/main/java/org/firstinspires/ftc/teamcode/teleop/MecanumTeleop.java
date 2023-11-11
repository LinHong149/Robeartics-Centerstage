package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Mecanum Teleop", group="Mecanum")
public class MecanumTeleop extends LinearOpMode {

    HardwareMecanum robot = new HardwareMecanum();

    boolean clawOpen = false;
    boolean isOuttaking = false;

    double lIntake = 0.77;
    double rIntake = 0.29;
    double lOuttake = 0.16;
    double rOuttake = 0.90;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for PLAY
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            // Motors: driving
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);

            double frontLeftPower = (drive + strafe + rotate) / denominator;
            double backLeftPower = (drive - strafe + rotate) / denominator;
            double frontRightPower = (drive - strafe - rotate) / denominator;
            double backRightPower = (drive + strafe - rotate) / denominator;

            robot.fr.setPower(frontRightPower);
            robot.fl.setPower(frontLeftPower);
            robot.br.setPower(backRightPower);
            robot.bl.setPower(backLeftPower);

            telemetry.addData("strafe","%.2f", strafe);
            telemetry.addData("drive","%.2f", drive);
            telemetry.addData("rotate","%.2f", rotate);


            // Slides
            while (gamepad1.right_trigger > 0) {
                telemetry.addData("dpad_up","active");
                telemetry.update();
                robot.lSlide.setPower(0.4);
                robot.rSlide.setPower(0.4);
            }
            while (gamepad1.left_trigger > 0) {
                robot.lSlide.setPower(-0.4);
                robot.rSlide.setPower(-0.4);
            }
            robot.lSlide.setPower(0);
            robot.rSlide.setPower(0);


            // Intake and outtake
            if (gamepad1.y) {
                telemetry.addData("Claw","active");
                telemetry.update();
                if (clawOpen) {
                    clawOpen = false;
                    robot.claw.setPosition(0.55); // Close the claw
                } else {
                    clawOpen = true;
                    robot.claw.setPosition(0.2); // Open the claw
                }
                sleep(200);
            }

            if (gamepad1.b) {
                if (isOuttaking) {
                    telemetry.addData("Extention","intake");
                    telemetry.update();
                    isOuttaking = false;
                    robot.wrist.setPosition(0.72); // Retract
                    robot.lArm.setPosition(lIntake); // Parallel
                    robot.rArm.setPosition(rIntake);
                } else {
                    telemetry.addData("Extention","outtake");
                    telemetry.update();
                    isOuttaking = true;
                    robot.wrist.setPosition(0.0); //Extend
                    robot.lArm.setPosition(lOuttake);
                    robot.rArm.setPosition(rOuttake);
                }
                sleep(200);
            }
            telemetry.update();
        }
    }
}



