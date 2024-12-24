package org.firstinspires.ftc.teamcode.commands.util;

public class Commands {
    public void runBlocking(Command command) {
        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
           running = command.run();
        }
    }
}
