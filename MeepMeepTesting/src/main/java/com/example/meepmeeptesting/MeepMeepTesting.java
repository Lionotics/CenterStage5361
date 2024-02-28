package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static final Pose2d RED_FRONTSTAGE_START = new Pose2d(12, -62, Math.toRadians(90.00));
    public static final Pose2d BLUE_FRONTSTAGE_START = mirror(RED_FRONTSTAGE_START);

    public static final Pose2d RED_BACKSTAGE_START = mirrorSide(RED_FRONTSTAGE_START).minus(new Pose2d(2,0,0));
    public static final Pose2d BLUE_BACKSTAGE_START = mirror(RED_BACKSTAGE_START);


    // Spike Marks

    // Stage side
    public static final Pose2d RED_RIGHT_LEFT_SPIKEMARK = new Pose2d(7,-36,Math.toRadians(135));
    public static final Pose2d RED_RIGHT_CENTER_SPIKEMARK = new Pose2d(13.5,-32,Math.toRadians(90));
    public static final Pose2d RED_RIGHT_RIGHT_SPIKEMARK = new Pose2d(23,-38,Math.toRadians(90));

    public static final Pose2d BLUE_LEFT_RIGHT_SPIKEMARK = mirror(RED_RIGHT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_CENTER_SPIKEMARK = mirror(RED_RIGHT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_LEFT_SPIKEMARK = mirror(RED_RIGHT_RIGHT_SPIKEMARK);

    public static Pose2d mirrorSide(Pose2d pose){
        return new Pose2d(-pose.getX() - 24 ,pose.getY(),pose.getHeading());
    }

    // Wall side
    public static final Pose2d RED_LEFT_LEFT_SPIKEMARK = new Pose2d(-46,-42,Math.toRadians(90));
    public static final Pose2d RED_LEFT_CENTER_SPIKEMARK = new Pose2d(-43.5,-32,Math.toRadians(45));
    public static final Pose2d RED_LEFT_RIGHT_SPIKEMARK = new Pose2d(-31,-36,Math.toRadians(45));

    public static final Pose2d BLUE_RIGHT_RIGHT_SPIKEMARK = mirror(RED_LEFT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_RIGHT_CENTER_SPIKEMARK = mirror(RED_LEFT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_RIGHT_LEFT_SPIKEMARK = mirror(RED_LEFT_RIGHT_SPIKEMARK);

    // Auto midpoints
    public static final Pose2d RED_LEFT_MIDPOINT = new Pose2d(-55,-12,0);
    public static final Pose2d RED_LEFT_LEFT_EXTRA_MIDPOINT = new Pose2d(-37.5, -38,Math.toRadians(90));
    public static final Pose2d BLUE_RIGHT_MIDPOINT = mirror(RED_LEFT_MIDPOINT);
    public static final Pose2d BLUE_RIGHT_RIGHT_EXTRA_MIDPOINT = mirror(RED_LEFT_LEFT_EXTRA_MIDPOINT);


    // Stage placing positions
    public static final Pose2d RED_LEFT_STAGE = new Pose2d(52,-26.25,0);
    public static final Pose2d RED_CENTER_STAGE = new Pose2d(52,-35,0);
    public static final Pose2d RED_RIGHT_STAGE = new Pose2d(52,-42.75,0);

    public static final Pose2d BLUE_RIGHT_STAGE = mirror(RED_LEFT_STAGE);
    public static final Pose2d BLUE_CENTER_STAGE = mirror(RED_CENTER_STAGE).plus(new Pose2d(0,1,0));
    public static final Pose2d BLUE_LEFT_STAGE = mirror(RED_RIGHT_STAGE);

    // Parking positions
    public static final Pose2d RED_PARK_EDGE_WAYPOINT = new Pose2d(45,-59,Math.toRadians(90));
    public static final Pose2d RED_PARK_EDGE = new Pose2d(53,-60,Math.toRadians(90));
    public static final Pose2d RED_STAGE_PARK = new Pose2d(45,-35,Math.toRadians(90));
    public static final Pose2d BLUE_PARK_EDGE = mirror(RED_PARK_EDGE);

    public static Pose2d mirror(Pose2d pose){
        return new Pose2d(pose.getX(),-pose.getY(),-pose.getHeading());
    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)

                                // RED CLOSE

                                // LEFT
//                                .forward(15)
//                                .splineTo(new Vector2d(RED_RIGHT_LEFT_SPIKEMARK.getX(),RED_RIGHT_LEFT_SPIKEMARK.getY()),RED_RIGHT_LEFT_SPIKEMARK.getHeading())
//                                .back(7)
//                                .lineToSplineHeading(RED_LEFT_STAGE)
//                                .waitSeconds(1)
//                                .setReversed(true)
//                                .splineTo(new Vector2d(RED_PARK_EDGE_WAYPOINT.getX(),RED_PARK_EDGE_WAYPOINT.getY()),RED_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(RED_PARK_EDGE.getX(),RED_PARK_EDGE.getY()),RED_PARK_EDGE.getHeading())
//                                .setReversed(false)




                                // CENTER
//                                .splineTo(new Vector2d(RED_RIGHT_CENTER_SPIKEMARK.getX(),RED_RIGHT_CENTER_SPIKEMARK.getY()),RED_RIGHT_CENTER_SPIKEMARK.getHeading())
//                                .back(7)
//                                .lineToSplineHeading(RED_CENTER_STAGE)
//                                .waitSeconds(1)
//                                .setReversed(true)
//                                .splineTo(new Vector2d(RED_PARK_EDGE_WAYPOINT.getX(),RED_PARK_EDGE_WAYPOINT.getY()),RED_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(RED_PARK_EDGE.getX(),RED_PARK_EDGE.getY()),RED_PARK_EDGE.getHeading())
//                                .setReversed(false)



                                // RIGHT
//                                .splineTo(new Vector2d(RED_RIGHT_RIGHT_SPIKEMARK.getX(),RED_RIGHT_RIGHT_SPIKEMARK.getY()),RED_RIGHT_RIGHT_SPIKEMARK.getHeading())
//                                .back(7)
//                                .lineToSplineHeading(RED_RIGHT_STAGE)
//                                .waitSeconds(1)
//                                .setReversed(true)
//                                .splineTo(new Vector2d(RED_PARK_EDGE_WAYPOINT.getX(),RED_PARK_EDGE_WAYPOINT.getY()),RED_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(RED_PARK_EDGE.getX(),RED_PARK_EDGE.getY()),RED_PARK_EDGE.getHeading())
//                                .setReversed(false)


                                // RED FAR
                                // All of these are goof thanks to the truss

                                // RIGHT
//                                .splineTo(new Vector2d(RED_LEFT_RIGHT_SPIKEMARK.getX(), RED_LEFT_RIGHT_SPIKEMARK.getY()), RED_LEFT_RIGHT_SPIKEMARK.getHeading())
//                                .back(7)
//                                .lineToSplineHeading(RED_LEFT_MIDPOINT)
//                                .forward(70)
//                                .splineTo(new Vector2d(RED_RIGHT_STAGE.getX(), RED_RIGHT_STAGE.getY()), RED_RIGHT_STAGE.getHeading())
//                                .waitSeconds(1)
//                                .back(5)
//                                .lineToSplineHeading(RED_STAGE_PARK)


                                // LEFT
                                .splineTo(new Vector2d(RED_LEFT_LEFT_SPIKEMARK.getX(),RED_LEFT_LEFT_SPIKEMARK.getY()),RED_LEFT_LEFT_SPIKEMARK.getHeading())
                                .back(5)
                                .strafeLeft(12)
                                .forward(25)
                                .splineTo(new Vector2d(RED_LEFT_MIDPOINT.getX()+10,RED_LEFT_MIDPOINT.getY()),RED_LEFT_MIDPOINT.getHeading())
                                .forward(70)
                                .splineTo(new Vector2d(RED_LEFT_STAGE.getX(),RED_LEFT_STAGE.getY()),RED_LEFT_STAGE.getHeading())
                                .waitSeconds(1)
                                .back(5)
                                .lineToSplineHeading(RED_STAGE_PARK)

                                // CENTER
//                                .lineToSplineHeading(RED_LEFT_CENTER_SPIKEMARK)
//                                .back(6)
//                                .lineToSplineHeading(RED_LEFT_MIDPOINT)
//                                .forward(70)
//                                .splineTo(new Vector2d(RED_CENTER_STAGE.getX(),RED_CENTER_STAGE.getY()),RED_CENTER_STAGE.getHeading())
//                                .waitSeconds(1)
//                                .back(5)
//                                .lineToSplineHeading(RED_STAGE_PARK)


                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}