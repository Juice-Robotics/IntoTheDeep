package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;

/**
 * Guardian of the pose.<br/>
 * <small>(Keeps track of the pose of the robot in between opmodes.)</small>
 */
public class PoseKeeper {
    /**
     * The current pose that the Keeper is aware of.
     */
    public static Pose2d currentPose = new Pose2d(0,0,0);
    /**
     * The history of the poses the Keeper was entrusted to guard.
     */
    public static ArrayList<Pose2d> poseHistory = new ArrayList<>();

    /**
     * Set the pose of the robot in the Keeper and add it to the Keeper's history
     * @param pose Pose of the robot that you want to save
     */
    public static void set(Pose2d pose) {
        currentPose = pose;
        poseHistory.add(pose);
    }

    /**
     * Get the Keeper's latest pose of the robot
     * @return Latest pose update of the robot
     */
    public static Pose2d get() {
        return currentPose;
    }

    /**
     * Get a specific pose from the Keeper's history
     * @param index The index of the pose you want
     * @return The pose of the robot at that index of the Keeper's history
     */
    public static Pose2d get(int index) {
        return poseHistory.get(index);
    }

    /**
     * Get the full Keeper's history
     * @return <code>ArrayList</code> of all the poses in the Keeper's history
     */
    public static ArrayList<Pose2d> history() {
        return poseHistory;
    }
}
