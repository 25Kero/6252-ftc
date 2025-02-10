package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class trajectory {
    private List<Path> paths = new ArrayList<Path>();
    private ElapsedTime taskTimer = new ElapsedTime();
    public void addPath(Path path) {
        paths.add(path);
    }
    public void runRoute() {
        int index = 0;
        while (index < paths.size()) {
            Path currentPath = paths.get(index);
            currentPath.startCall();

            taskTimer.reset();
            while (!currentPath.isComplete) {
                currentPath.tick();
                if (taskTimer.time(TimeUnit.MILLISECONDS) > currentPath.runTime) {currentPath.isComplete = true;} // force interrupt
            }
            currentPath.endCall();
            index++;
        }
    }
}
