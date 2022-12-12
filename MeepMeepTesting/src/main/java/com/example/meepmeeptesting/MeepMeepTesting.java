package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(270)))

                                /**start to low junction**/
                                .lineToLinearHeading(new Pose2d(-28, -52, Math.toRadians(45)))
//                                .splineTo(new Vector2d(-34,-30), Math.toRadians(90))
//                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-35.5, -60, Math.toRadians(60)))
                                .lineToLinearHeading(new Pose2d(-35.5, 2, Math.toRadians(90)))
                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-40, -5, Math.toRadians(90)), 1)
                                .splineToSplineHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3)





//                                .splineToSplineHeading(new Pose2d(-35.5, -25, Math.toRadians(45)), 1.5)
//                                .splineToSplineHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3.2)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-61,-10), Math.toRadians(180))
                                .setReversed(true)
//                                .splineTo(new Vector2d(-52, 0.5), Math.toRadians(0))
//                                .splineTo(new Vector2d(-27, -9.5), Math.toRadians(225))

                                .splineToSplineHeading(new Pose2d(-26.5, -18.5, Math.toRadians(315)), 5.5)
                                .splineToLinearHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3.2)
                                .splineToSplineHeading(new Pose2d(-26.5, -18.5, Math.toRadians(315)), 5.5)
                                //zone 1
//                                .splineToSplineHeading(new Pose2d(-60, -4, Math.toRadians(0)), 3.15)


                                .build()
                );

        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-26.5, -19.5, Math.toRadians(315)))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-57, -10, Math.toRadians(270)), 3.15)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}