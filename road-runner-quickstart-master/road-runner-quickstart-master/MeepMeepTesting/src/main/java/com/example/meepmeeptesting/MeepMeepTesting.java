package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(175), Math.toRadians(175), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35, -15, Math.toRadians(160)))
//                                .lineToLinearHeading((new Pose2d(34, -12, Math.toRadians(0))))
//                                .lineToLinearHeading((new Pose2d(40, -12, Math.toRadians(0))))
//                                .lineToLinearHeading((new Pose2d(34, -12, Math.toRadians(0))))
//                                .lineToLinearHeading((new Pose2d(30, -2, Math.toRadians(-10))))
//                                .lineToLinearHeading((new Pose2d(35, -35, Math.toRadians(0))))
//                                .lineToLinearHeading((new Pose2d(10, -35, Math.toRadians(0))))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}