package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(270)))
                                    // Traj 1
                                    .lineToLinearHeading(new Pose2d(-50, -46, Math.toRadians(0))) // x value (-58, -50, -36)
                                    .strafeLeft(16)
                                    .addDisplacementMarker(() -> {
                                        // Open Claw
                                    })
                                    .strafeLeft(18)
                                    .forward(78) // (86, 78, 64)
                                    .lineToSplineHeading(new Pose2d(48, -29, Math.toRadians(180)))

                                    .addDisplacementMarker(() -> {}) // Traj 2

                                    .lineToConstantHeading(new Vector2d(28, -12))
                                    .forward(86)

                                    .addDisplacementMarker(() -> {}) // Trag 3
                                    .back(86)
                                    .lineTo(new Vector2d(48, -30))

                                    .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}