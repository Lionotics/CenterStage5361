package com.example.meepmeep;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        // Note: these are trajectories from last year that I copied over for testing that I installed MeepMeep right, they are useless.
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40,40,toRadians(180),toRadians(180),15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35.25, -61.5, toRadians(90)))
                                        .forward(20)
                                        .splineTo(new Vector2d(-31,-30),toRadians(45))
                                        .forward(2)
                                        .lineToLinearHeading(new Pose2d(-39,-35,toRadians(90)))
                                        .forward(20)
                                        .splineTo(new Vector2d(-50,-10),toRadians(180))
                                        .forward(12.1)
                                        .back(13)
                                        .lineToLinearHeading(new Pose2d(-34.5,-15,toRadians(-45)))
                                        .waitSeconds(0.2)
                                        .forward(3.5)
                                        .back(10)
                                        .lineToSplineHeading(new Pose2d(-62.1,-10,toRadians(180)))

                                        .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot.setDimensions(17,17))
                .start();
    }
}