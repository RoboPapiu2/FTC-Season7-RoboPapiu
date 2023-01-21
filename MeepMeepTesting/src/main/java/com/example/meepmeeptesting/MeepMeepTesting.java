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
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(-28, -52, Math.toRadians(45))) //drop cone 1

                                .lineToLinearHeading(new Pose2d(-35.5, -55, Math.toRadians(60))) //pushcone 1
                                .lineToLinearHeading(new Pose2d(-35.5, -8, Math.toRadians(90))) //2
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-64.5, -10, Math.toRadians(180)), 3) //3
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(-27.5, -17.5, Math.toRadians(315)), 5.5) //traj_3repeat
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(-64.5, -10, Math.toRadians(180)), 3.2) // traj4_1
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(-27.5, -17.5, Math.toRadians(315)), 5.5) //traj_3repeat
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(-64.5, -10, Math.toRadians(180)), 3.2) // traj4_1
                                .waitSeconds(1.5)

                                .setReversed(false)

                                //zone 1
                                .lineToLinearHeading(new Pose2d(-48, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(270)))

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
                                .splineToLinearHeading(new Pose2d(64.5, -13, Math.toRadians(0)), 0) //3
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(26.5, -17.5, Math.toRadians(225)), 4.1) //traj_3repeat
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(64.5, -13, Math.toRadians(0)), -0) // traj4_1
                                .waitSeconds(1.5)

                                .splineToSplineHeading(new Pose2d(26.5, -17.5, Math.toRadians(225)), 4.1) //traj_3repeat
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(64.5, -13, Math.toRadians(0)), -0) // traj4_1
                                .waitSeconds(1.5)

                                .setReversed(false)

                                //zone 1
                                .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(57, -10, Math.toRadians(270)))

                                .build()
                );

        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39, 30, Math.toRadians(220), Math.toRadians(60), 14.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -61, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(28, -52, Math.toRadians(135))) //drop cone 1
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

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botStanga)
                .addEntity(botDreapta)
                .start();
    }
}