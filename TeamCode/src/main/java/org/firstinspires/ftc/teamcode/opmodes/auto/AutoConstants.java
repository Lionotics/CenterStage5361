package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
@Config
public class AutoConstants {

    // Start Positions
    public static final Pose2d RED_FRONTSTAGE_START = new Pose2d(12, -62, Math.toRadians(90.00)); // CHECK THIS
    public static final Pose2d BLUE_FRONTSTAGE_START = mirror(RED_FRONTSTAGE_START); // CHECK THIS

    public static final Pose2d RED_BACKSTAGE_START = mirrorSide(RED_FRONTSTAGE_START).plus(new Pose2d(1,0,0));
    public static final Pose2d BLUE_BACKSTAGE_START = mirror(RED_BACKSTAGE_START);


    // Spike Marks

    // Stage side
    public static final Pose2d RED_RIGHT_LEFT_SPIKEMARK = new Pose2d(7.5,-36,Math.toRadians(135));
    public static final Pose2d RED_RIGHT_CENTER_SPIKEMARK = new Pose2d(13.5,-33,Math.toRadians(90));
    public static final Pose2d RED_RIGHT_RIGHT_SPIKEMARK = new Pose2d(22,-38,Math.toRadians(90));

    public static final Pose2d BLUE_LEFT_RIGHT_SPIKEMARK = mirror(RED_RIGHT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_CENTER_SPIKEMARK = mirror(RED_RIGHT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_LEFT_LEFT_SPIKEMARK = mirror(RED_RIGHT_RIGHT_SPIKEMARK);

    // Wall side
    public static final Pose2d RED_LEFT_LEFT_SPIKEMARK = new Pose2d(-46,-42,Math.toRadians(90));
    public static final Pose2d RED_LEFT_CENTER_SPIKEMARK = new Pose2d(-43.5,-30,Math.toRadians(45));
    public static final Pose2d RED_LEFT_RIGHT_SPIKEMARK = new Pose2d(-31,-36,Math.toRadians(45));

    public static final Pose2d BLUE_RIGHT_RIGHT_SPIKEMARK = mirror(RED_LEFT_LEFT_SPIKEMARK);
    public static final Pose2d BLUE_RIGHT_CENTER_SPIKEMARK = mirror(RED_LEFT_CENTER_SPIKEMARK);
    public static final Pose2d BLUE_RIGHT_LEFT_SPIKEMARK = mirror(RED_LEFT_RIGHT_SPIKEMARK);

    // Auto midpoints
    public static final Pose2d RED_LEFT_MIDPOINT = new Pose2d(-55,-12,0);
    public static final Pose2d RED_LEFT_LEFT_EXTRA_MIDPOINT = new Pose2d(-34.5, -38,Math.toRadians(90)); // minus two
    public static final Pose2d BLUE_RIGHT_MIDPOINT = mirror(RED_LEFT_MIDPOINT);
    public static final Pose2d BLUE_RIGHT_RIGHT_EXTRA_MIDPOINT = mirror(RED_LEFT_LEFT_EXTRA_MIDPOINT);


    // Stage placing positions
    public static final Pose2d RED_LEFT_STAGE = new Pose2d(52,-27.5,0);
    public static final Pose2d RED_CENTER_STAGE = new Pose2d(52,-34,0);
    public static final Pose2d RED_RIGHT_STAGE = new Pose2d(52,-41,0);

    public static final Pose2d BLUE_RIGHT_STAGE = mirror(RED_LEFT_STAGE);
    public static final Pose2d BLUE_CENTER_STAGE = mirror(RED_CENTER_STAGE);
    public static final Pose2d BLUE_LEFT_STAGE = mirror(RED_RIGHT_STAGE);
    
    // Parking positions
    public static final Pose2d RED_PARK_EDGE_WAYPOINT = new Pose2d(48,-59,Math.toRadians(90));
    public static final Pose2d RED_PARK_EDGE = new Pose2d(53,-57,Math.toRadians(90));
    public static final Pose2d RED_STAGE_PARK = new Pose2d(45,-35,Math.toRadians(90));

    public static final Pose2d BLUE_PARK_EDGE = mirror(RED_PARK_EDGE);
    public static final Pose2d BLUE_PARK_EDGE_WAYPOINT = mirror(RED_PARK_EDGE_WAYPOINT);
    public static final Pose2d BLUE_STAGE_PARK = mirror(RED_STAGE_PARK);


    public static Pose2d mirror(Pose2d pose){
        return new Pose2d(pose.getX(),-pose.getY(),-pose.getHeading());
    }
    public static Pose2d mirrorSide(Pose2d pose){
        return new Pose2d(-pose.getX() - 24 ,pose.getY(),pose.getHeading());
    }

}
