package org.firstinspires.ftc.teamcode.commands.primitives;

import org.firstinspires.ftc.teamcode.commands.util.Command;

import java.util.Arrays;

public class SequentialCommand implements Command {
    Command[] queue;
    public SequentialCommand(Command... cmds) {
        queue = cmds;
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

    private Command[] dropFirstElement(Command[] array) {
        if (array == null || array.length == 0) {
            return new Command[0];
        }

        Command[] newArray = new Command[array.length - 1];
        System.arraycopy(array, 1, newArray, 0, newArray.length);
        return newArray;
    }
}
