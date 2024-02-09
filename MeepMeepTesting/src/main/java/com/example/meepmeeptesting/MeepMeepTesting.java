package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redLeftRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.6, 17)
                .followTrajectorySequence(drive -> {

                    Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));


                    return drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-35, -30, Math.toRadians(0)), Math.toRadians(90))

                            .addDisplacementMarker(() -> {
                                // drop purple pixel
                            })

                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(-30, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(180)), Math.toRadians(270))
//                            .splineToConstantHeading(new Vector2d(-30, -9), Math.toRadians(0))
//                            .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(270))

                            .addDisplacementMarker(() -> {
                                // drop yellow pixel
                            })

//                            .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(190))
                            .setReversed(false)
                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // grab from pixel stack
                            })

                            .setReversed(true)
//                            .setTangent(0.2)
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

                            .build();

                });

        RoadRunnerBotEntity redLeftLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {

                    Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));


                    return drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-34, -30, Math.toRadians(180)), Math.toRadians(90))

                            .addDisplacementMarker(() -> {
                                // drop purple pixel
                            })

                            .setTangent(1.5)
//                            .splineToConstantHeading(new Vector2d(-26, -9), Math.toRadians(0))
//                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(180)), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(-26, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -37), Math.toRadians(270))

                            .addDisplacementMarker(() -> {
                                // drop yellow pixel
                            })

//                            .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(190))
                            .setReversed(false)
                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // grab from pixel stack
                            })

                            .setReversed(true)
//                            .setTangent(0.2)
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

                            .build();

                });

        RoadRunnerBotEntity redLeftMiddle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {

                    Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));


                    return drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-52, -26, Math.toRadians(0)), Math.toRadians(90))

                            .addDisplacementMarker(() -> {
                                // drop purple pixel
                            })

                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(-40, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(180)), Math.toRadians(270))
//                            .splineToConstantHeading(new Vector2d(-44, -9), Math.toRadians(0))
//                            .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(270))

                            .addDisplacementMarker(() -> {
                                // drop yellow pixel
                            })

//                            .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(190))
                            .setReversed(false)
                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // grab from pixel stack
                            })

                            .setReversed(true)
//                            .setTangent(0.2)
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

                            .build();

                });


        RoadRunnerBotEntity redRightRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {

                    Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));


                    return drive.trajectorySequenceBuilder(startPose)
                            //RED RIGHT RIGHT
                            .splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(180)), Math.toRadians(90))

                            .addDisplacementMarker(() -> {
                                // drop purple pixel
                            })

                            .setReversed(true)
//                            .splineToSplineHeading(new Pose2d(44, -43, Math.toRadians(0)), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(44, -43), Math.toRadians(270))

                            .addDisplacementMarker(() -> {
                                // drop yellow pixel
                            })

//                            .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(190))
                            .setReversed(false)
                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // grab from pixel stack
                            })

                            .setReversed(true)
//                            .setTangent(0.2)
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

                            .build();

                });

        RoadRunnerBotEntity redRightLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {

                    Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));


                    return drive.trajectorySequenceBuilder(startPose)
                            //RED RIGHT LEFT
                            .splineToLinearHeading(new Pose2d(16, -28, Math.toRadians(180)), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // drop purple pixel
                            })

                            .setReversed(true)
//                            .splineToSplineHeading(new Pose2d(44, -31, Math.toRadians(0)), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(44, -31), Math.toRadians(90))

                            .addDisplacementMarker(() -> {
                                // drop yellow pixel
                            })

//                            .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(190))
                            .setReversed(false)
                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // grab from pixel stack
                            })

                            .setReversed(true)
//                            .setTangent(0.2)
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

                            .build();

                });

        RoadRunnerBotEntity redRightMiddle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {

                    Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));


                    return drive.trajectorySequenceBuilder(startPose)
                            // RED RIGHT MIDDLE
                            .splineToLinearHeading(new Pose2d(16, -36, Math.toRadians(90)), Math.toRadians(0))

                            .addDisplacementMarker(() -> {
                                // drop purple pixel
                            })
                            .splineToSplineHeading(new Pose2d(44, -37, Math.toRadians(180)), Math.toRadians(90))
//                            .splineToConstantHeading(new Vector2d(44, -37), Math.toRadians(90))

                            .addDisplacementMarker(() -> {
                                // drop yellow pixel
                            })

//                            .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(180)), Math.toRadians(190))
                            .setReversed(false)
                            .setTangent(1.5)
                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(180))

                            .addDisplacementMarker(() -> {
                                // grab from pixel stack
                            })

                            .setReversed(true)
//                            .setTangent(0.2)
//                            .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(270))

                            .splineToConstantHeading(new Vector2d(16, -9), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(44, -36), Math.toRadians(270))

                            .addDisplacementMarker(() -> {
                                // drop pixel
                            })


//                            PARKING
                            .splineToLinearHeading(new Pose2d(52, -59, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(56, -59, Math.toRadians(180)), Math.toRadians(0))


                            .build();

                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redLeftRight)
//                .addEntity(redLeftLeft)
//                .addEntity(redLeftMiddle)
//                .addEntity(redRightRight)
//                .addEntity(redRightLeft)
                .addEntity(redRightMiddle)
                .start();

    }
}