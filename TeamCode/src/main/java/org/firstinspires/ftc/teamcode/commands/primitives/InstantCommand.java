package org.firstinspires.ftc.teamcode.commands.primitives;

import org.firstinspires.ftc.teamcode.commands.util.Command;

public class InstantCommand implements Command {

    public Runnable func;
    public InstantCommand(Runnable lambda) {
        func = lambda;
    }

    public boolean run() {
        func.run();
        return false;
    }
}
