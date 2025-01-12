package org.firstinspires.ftc.teamcode.commands.primitives;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.util.Command;

import java.util.concurrent.TimeUnit;

public class SleepCommand implements Command {
    ElapsedTime timer;
    double targetTime;
    boolean hasStarted = false;
    public SleepCommand(double seconds) {
        targetTime = seconds;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public boolean run() {
        if (!hasStarted) {
            timer.reset();
            hasStarted = true;
            return true;
        }
        return timer.time(TimeUnit.MILLISECONDS) <= (targetTime*1000);
    }
}
