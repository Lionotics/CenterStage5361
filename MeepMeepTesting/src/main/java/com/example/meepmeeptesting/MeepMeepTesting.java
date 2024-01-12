package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static final Pose2d RED_RIGHT_START = new Pose2d(11.5, -62, Math.toRadians(90.00));
    public static final Pose2d BLUE_LEFT_START = mirror(RED_RIGHT_START);

    public static final Pose2d RED_LEFT_START = mirrorSide(RED_RIGHT_START).minus(new Pose2d(2,0,0));
    public static final Pose2d BLUE_RIGHT_START = mirror(RED_LEFT_START);



    public static final Pose2d RED_RIGHT_LEFT_SPIKEMARK = new Pose2d(10,-38,Math.toRadians(135));
    public static final Pose2d RED_RIGHT_CENTER_SPIKEMARK = new Pose2d(11.5,-38,Math.toRadians(90));
    public static final Pose2d RED_RIGHT_RIGHT_SPIKEMARK = new Pose2d(11.5,-38,Math.toRadians(45));

    public static final Pose2d RED_LEFT_LEFT_SPIKEMARK = mirrorSide(RED_RIGHT_LEFT_SPIKEMARK).minus(new Pose2d(5,0,0));
    public static final Pose2d RED_LEFT_CENTER_SPIKEMARK = mirrorSide(RED_RIGHT_CENTER_SPIKEMARK).minus(new Pose2d(16,-14,Math.toRadians(90)));
    public static final Pose2d RED_LEFT_RIGHT_SPIKEMARK = mirrorSide(RED_RIGHT_RIGHT_SPIKEMARK);


    public static final Pose2d BLUE_LEFT_RIGHT_SPIKEMARK = mirror(RED_RIGHT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_CENTER_SPIKEMARK = mirror(RED_RIGHT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_LEFT_SPIKEMARK = mirror(RED_RIGHT_RIGHT_SPIKEMARK);

    public static final Pose2d RED_LEFT_MIDPOINT = new Pose2d(-35,-10,0);
    public static final Pose2d RED_LEFT_LEFT_EXTRA_MIDPOINT = new Pose2d(-36, -38,Math.toRadians(90));


    public static final Pose2d RED_LEFT_STAGE = new Pose2d(52,-26,0);
    public static final Pose2d RED_CENTER_STAGE = new Pose2d(52,-35,0);
    public static final Pose2d RED_RIGHT_STAGE = new Pose2d(52,-42,0);

    public static final Pose2d BLUE_RIGHT_STAGE = mirror(RED_LEFT_STAGE);
    public static final Pose2d BLUE_CENTER_STAGE = mirror(RED_CENTER_STAGE);
    public static final Pose2d BLUE_LEFT_STAGE = mirror(RED_RIGHT_STAGE);

    public static final Pose2d RED_PARK_EDGE = new Pose2d(53,-60,Math.toRadians(90));
    public static final Pose2d BLUE_PARK_EDGE = mirror(RED_PARK_EDGE);

    public static Pose2d mirror(Pose2d pose){
        return new Pose2d(pose.getX(),-pose.getY(),-pose.getHeading());
    }
    public static Pose2d mirrorSide(Pose2d pose){
        return new Pose2d(-pose.getX() - 24 ,pose.getY(),pose.getHeading());
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 16.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_LEFT_START)
                                // RIGHT
//                                .forward(15)
//                                .lineToSplineHeading(RED_LEFT_RIGHT_SPIKEMARK)
//                                .waitSeconds(1)
//                                .strafeLeft(6)
//                                .lineToSplineHeading(RED_LEFT_MIDPOINT)
////                                Wait 15ish seconds here for other robot?
//                                .forward(55)
//                                .splineTo(new Vector2d(RED_RIGHT_STAGE.getX(),RED_RIGHT_STAGE.getY()),RED_RIGHT_STAGE.getHeading())

//                                // CENTER
//                                .forward(15)
//                                .lineToSplineHeading(RED_LEFT_CENTER_SPIKEMARK)
//                                .waitSeconds(1)
//                                .strafeLeft(10)
//                                .lineToSplineHeading(RED_LEFT_MIDPOINT)
////                                Wait 15ish seconds here for other robot?
//                                .forward(55)
//                                .splineTo(new Vector2d(RED_CENTER_STAGE.getX(),RED_CENTER_STAGE.getY()),RED_CENTER_STAGE.getHeading())

                                // LEFT
                                .strafeLeft(3)
                                .forward(15)
                                .lineToSplineHeading(RED_LEFT_LEFT_SPIKEMARK)
                                .waitSeconds(1)
//                                .back(2)
                                .lineToSplineHeading(RED_LEFT_LEFT_EXTRA_MIDPOINT)
                                .forward(20)
                                .lineToSplineHeading(RED_LEFT_MIDPOINT)
//                                Wait 15ish seconds here for other robot?
//                                .forward(55)
//                                .splineTo(new Vector2d(RED_LEFT_STAGE.getX(),RED_LEFT_STAGE.getY()),RED_LEFT_STAGE.getHeading())
                                .waitSeconds(2)



                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}