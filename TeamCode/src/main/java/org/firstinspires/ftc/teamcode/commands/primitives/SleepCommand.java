package org.firstinspires.ftc.teamcode.commands.primitives;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.util.Command;

import java.util.concurrent.TimeUnit;

public class SleepCommand implements Command {
    ElapsedTime timer;
    double targetTime;
    public SleepCommand(double seconds) {
        targetTime = seconds;
        timer = new ElapsedTime();
    }

    @Override
    public boolean run() {
        return timer.time(TimeUnit.SECONDS) < targetTime;
    }
}
