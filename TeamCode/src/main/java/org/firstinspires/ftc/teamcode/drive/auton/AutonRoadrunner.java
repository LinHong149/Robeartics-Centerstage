package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonRoadrunner", group = "auton")
public class AutonRoadrunner extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;


        // Trajectories
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);
        Trajectory spikeMarkLeft = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();

        drive.followTrajectory(spikeMarkLeft);

    }
}
