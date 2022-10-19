package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class AutonomusUtil {
    public TrajectorySequenceBuilder AutonomusCone(TrajectorySequenceBuilder tsb, int times, Pose2d liftConePose, Pose2d placeConePose, double liftEndHeading, double placeEndHeading) {
        for (int i = 0; i < times; i++) {
            tsb
                    .setReversed(true)
                    .splineToSplineHeading(liftConePose, Math.toRadians(liftEndHeading))
                    .setReversed(false)
                    .addDisplacementMarker(() -> {
                        // TODO: Lift cone
                    })
                    .splineToSplineHeading(placeConePose, Math.toRadians(placeEndHeading))
                    .addDisplacementMarker(() -> {
                        // TODO: Place cone
                    });
        }
        return tsb;
    }

    public TrajectorySequenceBuilder AutonoumusPark(TrajectorySequenceBuilder tsb, int parkPosition) {
        switch (parkPosition) {
            case 1: {

            }
            case 2: {

            }
            case 3: {

            }
        }
        return tsb;
    }
}
