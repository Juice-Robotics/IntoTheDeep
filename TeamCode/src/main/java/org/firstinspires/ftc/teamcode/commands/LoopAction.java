package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.concurrent.Callable;

public class LoopAction implements Action {
    Runnable function;
    Callable<Boolean> isStopRequested;
    boolean manualStop;

    /**
     * Action that just runs the <code>lambda</code> in a constant loop with no end (until it does end)
     * @param lambda function you want running in a loop
     * @param isStopRequested the <code>isStopRequested()</code> callable provided in the opmode. Kills the loop when the opmode ends
     */
    public LoopAction(Runnable lambda, Callable<Boolean> isStopRequested) {
        this.isStopRequested = isStopRequested;
        function = lambda;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
           if (isStopRequested.call() || manualStop) return false;
        } catch (Exception e) {
           throw new RuntimeException(e);
        }
        function.run();
        return true;
    }

    /**
     * Kills the loop manually (before the opmode ends)
     */
    public void kill() {
        manualStop = true;
    }
}
