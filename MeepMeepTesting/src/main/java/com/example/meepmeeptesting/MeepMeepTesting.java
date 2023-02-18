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

        RoadRunnerBotEntity botStanga = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-64.5, -11, Math.toRadians(180)))

                                .setReversed(true)
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(-27.5, -17.2, Math.toRadians(315)), 5.5) //traj_3repeat
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(-50, -11, Math.toRadians(180)), 3.2)
                                .splineToSplineHeading(new Pose2d(-64.5, -11, Math.toRadians(180)), Math.toRadians(180)) // traj4_1

                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(-27.5, -17.5, Math.toRadians(315)), 5.5) //traj_3repeat
//                                .splineToSplineHeading(new Pose2d(-64.5, -10, Math.toRadians(180)), 3.2) // traj4_1
//                                .waitSeconds(1.5)

                                .setReversed(false)

                                //zone 1
                                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(-59, -13, Math.toRadians(270)))

                                .build()
                );

        RoadRunnerBotEntity botDreapta = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -61, Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(28, -52, Math.toRadians(135))) //drop cone 1

                                .lineToLinearHeading(new Pose2d(35.5, -55, Math.toRadians(125))) //pushcone 1
                                .lineToLinearHeading(new Pose2d(35.5, -8, Math.toRadians(90))) //2
                                .setReversed(true)
                                .setTangent(24)
                                .splineToSplineHeading(new Pose2d(48, -11, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(64.5, -11, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(27.5, -17.2, Math.toRadians(225)), 4) //traj_3repeat
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(40, -11, Math.toRadians(0)), -0)
                                .splineToSplineHeading(new Pose2d(64.5, -11, Math.toRadians(0)), -0) // traj4_1
                                .waitSeconds(0.1)

                                .splineToSplineHeading(new Pose2d(27.5, -17.2, Math.toRadians(225)), 4) //traj_3repeat
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(40, -11, Math.toRadians(0)), -0)
                                .splineToSplineHeading(new Pose2d(64.5, -11, Math.toRadians(0)), -0) // traj4_1
                                .waitSeconds(0.1)

                                .setReversed(false)

                                //zone 1
                                .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(57, -10, Math.toRadians(270)))

                                .build()
                );

        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -8, Math.toRadians(90)))
                                .setReversed(true)
                                .setTangent(24)
                                .splineToSplineHeading(new Pose2d(-48, -10, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-64.5, -10, Math.toRadians(180)), Math.toRadians(180))
                                .build()
                );
        RoadRunnerBotEntity testBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-28, -52, Math.toRadians(45))) //drop cone 1
                                .build()
                );

        RoadRunnerBotEntity stangaHighAll = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-35.5, -30, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-27.5, -4, Math.toRadians(45)), Math.toRadians(45))
                                /* traj 3 repeat */
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-50, -11, Math.toRadians(180)), 3.2)
                                .splineToSplineHeading(new Pose2d(-64.5, -11, Math.toRadians(180)), Math.toRadians(180))
                                //
                                .waitSeconds(0.1)
                                /* conesToMidJ */
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-27.5, -4, Math.toRadians(45)), 0.8)
//                                .waitSeconds(0.1)
                                /* parcare 1 */
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-50, -11, Math.toRadians(270)), 3.2)
//                                .splineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(270)), Math.toRadians(180))

                                /* parcare 2 */
//                                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(90)))

                                /* parcare 3 si middle high cone */
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35, -11, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-4, -17.2, Math.toRadians(315)), Math.toRadians(315))
                                .lineToLinearHeading(new Pose2d(-10, -13, Math.toRadians(270)))

                                .build()
                );


//        RoadRunnerBotEntity stangaAllMiddle = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
//                                .splineToSplineHeading(new Pose2d(-35.5, -45, Math.toRadians(90)), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(-27.5, -27.2, Math.toRadians(45)), Math.toRadians(45))
//                                .build()
//                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(botStanga)
                //.addEntity(botDreapta)
                //.addEntity(testBot)
                //.addEntity(stangaHighAll)
                //.addEntity(stangaAllMiddle)
                .start();
    }
}