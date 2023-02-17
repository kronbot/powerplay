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
                .setConstraints(52, 52, Math.toRadians(180), Math.toRadians(180), 11.25)
                .setDimensions(12,12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-31.90, -66.40, Math.toRadians(90.00)))
                                .splineToSplineHeading(new Pose2d(-29.50, -5.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-46.15, -12.26, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .splineToSplineHeading(new Pose2d(-61.00, -12.25, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-17.15, -7.80, Math.toRadians(135.00)), Math.toRadians(35.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-8.70, -12.25, Math.toRadians(180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-61.00, -12.25, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-17.15, -7.80, Math.toRadians(135.00)), Math.toRadians(35.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-8.70, -12.25, Math.toRadians(180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-61.00, -12.25, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-17.15, -7.80, Math.toRadians(135.00)), Math.toRadians(35.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-8.70, -12.25, Math.toRadians(180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-61.00, -12.25, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-17.15, -7.80, Math.toRadians(135.00)), Math.toRadians(35.00))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-8.70, -12.25, Math.toRadians(180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-61.00, -12.25, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}