package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class AutonomusUtil {
    /**
     * Adds multiple lift and place trajectory's
     * @param tsb TrajectorySequenceBuilder it is adding new trajectory's to
     * @param times Number of lift and place trajectory's to add
     * @param liftConePose Position used for lifting cones
     * @param placeConePose Position used for placing cones
     * @param liftEndHeading End tangent for lifting cones(last value for splineToSplineHeading when lifting cones)
     * @param placeEndHeading End tangent for placing cones(last value for splineToSplineHeading when placing cones)
     * @return Returns the TrajectorySequenceBuilder with the added Trajectory's
     */
    public TrajectorySequenceBuilder AutonomusCone(TrajectorySequenceBuilder tsb, int times, Pose2d liftConePose, Pose2d placeConePose, double liftEndHeading, double placeEndHeading) {
        for (int i = 0; i < times; i++) {
            tsb
                    .setReversed(true)
                    /* TODO: Add getting ready for lifting the cone */
                    .splineToSplineHeading(liftConePose, Math.toRadians(liftEndHeading))
                    .setReversed(false)
                    .addDisplacementMarker(() -> {
                        // TODO: Lift the cone
                    })
                    /* TODO: Add getting ready for placing the cone */
                    .splineToSplineHeading(placeConePose, Math.toRadians(placeEndHeading))
                    .addDisplacementMarker(() -> {
                        // TODO: Place the cone
                    });
        }
        return tsb;
    }

    /**
     * Adds the position for automated parking at the end
     * @param tsb TrajectorySequenceBuilder it is adding the automated parking trajectory
     * @param parkPosition Int vector used that shows where to park
     * @return Returns the TrajectorySequenceBuilder with the added automated parking trajectory
     */
    public TrajectorySequenceBuilder AutonoumusPark(TrajectorySequenceBuilder tsb, int parkPosition) {
        switch (parkPosition) {
            case 2: {
                tsb.forward(28);
            }
            case 3: {
                tsb.forward(50);
            }
            default:
        }
        return tsb;
    }
}
