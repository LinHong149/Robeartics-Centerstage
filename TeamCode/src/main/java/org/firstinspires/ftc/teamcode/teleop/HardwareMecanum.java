package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMecanum {

    // Servos - define hardware objects
    public Servo claw = null;
    public Servo wrist = null;
    public Servo lArm = null;
    public Servo rArm = null;

    // Motors - define hardware objects
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor br = null;
    public DcMotor bl = null;
    public DcMotor lSlide = null;
    public DcMotor rSlide = null;

    // Local op mode members
    HardwareMap hwMap = null;

    // Constructor
    public HardwareMecanum() {}

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Servos
        claw = ahwMap.get(Servo.class, "claw");
        wrist = ahwMap.get(Servo.class, "wrist");
        lArm = ahwMap.get(Servo.class, "lArm");
        rArm = ahwMap.get(Servo.class, "rArm");

        wrist.setPosition(0.73);
        claw.setPosition(0.55); // Closed
        lArm.setPosition(0.77); // Intaking
        rArm.setPosition(0.29); // Intaking


        // Motors
        fr = ahwMap.get(DcMotor.class, "fr");
        fl = ahwMap.get(DcMotor.class, "fl");
        br = ahwMap.get(DcMotor.class, "br");
        bl = ahwMap.get(DcMotor.class, "bl");
        lSlide = ahwMap.get(DcMotor.class, "lSlide");
        rSlide = ahwMap.get(DcMotor.class, "rSlide");

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
        lSlide.setDirection(DcMotor.Direction.FORWARD);
        rSlide.setDirection(DcMotor.Direction.REVERSE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}












