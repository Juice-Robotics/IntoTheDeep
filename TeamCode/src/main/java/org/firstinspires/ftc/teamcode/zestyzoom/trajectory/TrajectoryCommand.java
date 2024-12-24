package org.firstinspires.ftc.teamcode.zestyzoom.trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.util.Command;

public class TrajectoryCommand implements Command {
    Command[] queue;
    Pose2D startPose;
    Pose2D endPose;
    public TrajectoryCommand(Pose2D startingPos, Pose2D endPos, Command... cmds) {
        queue = cmds;
        startPose = startingPos;
        endPose = endPos;
    }

    @Override
    public boolean run() {
        if (queue.length == 0) {
            return false;
        }
        if (queue[0].run()) {
            return true;
        } else {
            queue = dropFirstElement(queue);
            return queue[0].run();
        }
    }

    public Pose2D startPose() {
        return startPose;
    }

    public Pose2D endPose() {
        return endPose;
    }

    private Command[] dropFirstElement(Command[] array) {
        if (array == null || array.length == 0) {
            return new Command[0];
        }

        Command[] newArray = new Command[array.length - 1];
        System.arraycopy(array, 1, newArray, 0, newArray.length);
        return newArray;
    }
}
