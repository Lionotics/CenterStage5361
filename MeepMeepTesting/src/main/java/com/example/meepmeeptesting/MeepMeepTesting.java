package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static final Pose2d RED_RIGHT_START = new Pose2d(11.5, -62, Math.toRadians(90.00));
    public static final Pose2d BLUE_LEFT_START = mirror(RED_RIGHT_START);

    public static final Pose2d RED_RIGHT_LEFT_SPIKEMARK = new Pose2d(8,-40,Math.toRadians(135));
    public static final Pose2d RED_RIGHT_CENTER_SPIKEMARK = new Pose2d(11.5,-36,Math.toRadians(90));
    public static final Pose2d RED_RIGHT_RIGHT_SPIKEMARK = new Pose2d(13,-40,Math.toRadians(45));

    public static final Pose2d BLUE_LEFT_RIGHT_SPIKEMARK = mirror(RED_RIGHT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_CENTER_SPIKEMARK = mirror(RED_RIGHT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_LEFT_SPIKEMARK = mirror(RED_RIGHT_RIGHT_SPIKEMARK);

    public static final Pose2d RED_LEFT_STAGE = new Pose2d(49.5,-26,0);
    public static final Pose2d RED_CENTER_STAGE = new Pose2d(49.5,-34,0);
    public static final Pose2d RED_RIGHT_STAGE = new Pose2d(49.5,-42,0);

    public static final Pose2d RED_PARK = new Pose2d(53,-60,Math.toRadians(90));
    public static  final Pose2d BLUE_PARK = mirror(RED_PARK);


    public static final Pose2d BLUE_RIGHT_STAGE = mirror(RED_LEFT_STAGE);
    public static final Pose2d BLUE_CENTER_STAGE = mirror(RED_CENTER_STAGE);
    public static final Pose2d BLUE_LEFT_STAGE = mirror(RED_RIGHT_STAGE);


    public static Pose2d mirror(Pose2d pose){
        return new Pose2d(pose.getX(),-pose.getY(),-pose.getHeading());
    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 16.35)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_LEFT_START)
                                // LEFT
//                                .setReversed(false)
                                .forward(15)
                                .lineToSplineHeading(BLUE_LEFT_LEFT_SPIKEMARK)
                                .waitSeconds(1)
                                .strafeLeft(6)
                                .lineToSplineHeading(BLUE_LEFT_STAGE)
                                .strafeLeft(5)
                                .lineToSplineHeading(new Pose2d(53,60,Math.toRadians(-90)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}