package org.firstinspires.ftc.teamcode.commands.primitives;

import org.firstinspires.ftc.teamcode.commands.util.Command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class ParallelCommand implements Command {
    List<Command> queue;
    public ParallelCommand(Command... cmds) {
        queue = new ArrayList<Command>(Arrays.asList(cmds));
    }

    @Override
    public boolean run() {
        queue = queue.stream()
                .filter(Command::run)
                .collect(Collectors.toList());
        return !queue.isEmpty();
    }
}
