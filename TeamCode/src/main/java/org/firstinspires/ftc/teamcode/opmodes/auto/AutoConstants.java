package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
@Config
public class AutoConstants {
    // Start Positions
    public static final Pose2d RED_FRONTSTAGE_START = new Pose2d(10.5, -62, Math.toRadians(90.00));
    public static final Pose2d BLUE_FRONTSTAGE_START = mirror(RED_FRONTSTAGE_START);

    public static final Pose2d RED_BACKSTAGE_START = mirrorSide(RED_FRONTSTAGE_START).minus(new Pose2d(2,0,0));
    public static final Pose2d BLUE_BACKSTAGE_START = mirror(RED_BACKSTAGE_START);


    // Spike Marks

    // Stage side
    public static final Pose2d RED_RIGHT_LEFT_SPIKEMARK = new Pose2d(9,-38,Math.toRadians(135));
    public static final Pose2d RED_RIGHT_CENTER_SPIKEMARK = new Pose2d(11.5,-38,Math.toRadians(90));
    public static final Pose2d RED_RIGHT_RIGHT_SPIKEMARK = new Pose2d(11.5,-38,Math.toRadians(45));

    public static final Pose2d BLUE_LEFT_RIGHT_SPIKEMARK = mirror(RED_RIGHT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_CENTER_SPIKEMARK = mirror(RED_RIGHT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_LEFT_SPIKEMARK = mirror(RED_RIGHT_RIGHT_SPIKEMARK);

    // Wall side
    public static final Pose2d RED_LEFT_LEFT_SPIKEMARK = mirrorSide(RED_RIGHT_LEFT_SPIKEMARK).minus(new Pose2d(4,2,0));
    public static final Pose2d RED_LEFT_CENTER_SPIKEMARK = mirrorSide(RED_RIGHT_CENTER_SPIKEMARK).minus(new Pose2d(16,-14,Math.toRadians(90)));
    public static final Pose2d RED_LEFT_RIGHT_SPIKEMARK = mirrorSide(RED_RIGHT_RIGHT_SPIKEMARK);

    public static final Pose2d BLUE_RIGHT_RIGHT_SPIKEMARK = mirror(RED_LEFT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_RIGHT_CENTER_SPIKEMARK = mirror(RED_LEFT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_RIGHT_LEFT_SPIKEMARK = mirror(RED_LEFT_RIGHT_SPIKEMARK);

    // Auto midpoints
    public static final Pose2d RED_LEFT_MIDPOINT = new Pose2d(-37,-10,0);
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
    public static final Pose2d RED_PARK_EDGE = new Pose2d(53,-60,Math.toRadians(90));
    public static final Pose2d BLUE_PARK_EDGE = mirror(RED_PARK_EDGE);


    public static Pose2d mirror(Pose2d pose){
        return new Pose2d(pose.getX(),-pose.getY(),-pose.getHeading());
    }
    public static Pose2d mirrorSide(Pose2d pose){
        return new Pose2d(-pose.getX() - 24 ,pose.getY(),pose.getHeading());
    }

}
