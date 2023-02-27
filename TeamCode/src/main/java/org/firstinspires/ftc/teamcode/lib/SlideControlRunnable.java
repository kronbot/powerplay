package org.firstinspires.ftc.teamcode.lib;

public class SlideControlRunnable implements Runnable {
    private boolean running = true;
    private final SlideControl slideControl;

    public SlideControlRunnable(SlideControl slideControl) {
        this.slideControl = slideControl;
    }

    @Override
    public void run() {
        while (running) {
            slideControl.loop(true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public synchronized void stop() {
        running = false;
    }
}
