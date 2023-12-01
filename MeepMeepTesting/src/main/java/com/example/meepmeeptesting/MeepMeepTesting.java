package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(240), Math.toRadians(240), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, -62, Math.toRadians(90.00)))
                                // LEFT
//                                .setReversed(false)
                                .forward(15)
                                .lineToSplineHeading(new Pose2d(8,-40,Math.toRadians(135)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(50,-28,0))
                                .waitSeconds(1)
                                // 53 -42
//                                .waitSeconds(1)
//                                .splineTo(new Vector2d(45,-31),0)
                                // CENTER
//                                .setReversed(false)
//                                .lineToSplineHeading(new Pose2d(25,-23,Math.toRadians(0)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(45,-35,0))
                                // RIGHT
//                                .setReversed(false)
//                                .lineToSplineHeading(new Pose2d(35,-33,Math.toRadians(0)))
//                                .waitSeconds(1)
//                                .splineTo(new Vector2d(48,-42),0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}