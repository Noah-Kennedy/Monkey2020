package main;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferStrategy;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Main extends Canvas implements Runnable {

    public static final int WIDTH = 1280;
    public static final int HEIGHT = WIDTH * 9 / 16;
    private static final String TITLE = "Steering Behavior Prototype";
    private static final Color BACKGROUND_COLOR = Color.DARK_GRAY;
    private Thread thread;
    private boolean running = false;

    private static final int TICKS_PER_SECOND = 60;
    private static final long NANOSECONDS_PER_TICK = 1000000000 / TICKS_PER_SECOND;

    private static final Random rand = new Random(0);
    private static Path path;
    private static final List<Vehicle> vehicles = new ArrayList<>();

    public static void main(String[] args) {
        JFrame f = new JFrame(TITLE);
        Dimension d = new Dimension(WIDTH + 6, HEIGHT + 29);
        Main m = new Main();
        f.add(m);
        f.setSize(d);
        f.setPreferredSize(d);
        f.setMaximumSize(d);
        f.setMinimumSize(d);
        f.setResizable(false);
        f.setVisible(true);
        f.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        f.setLocationRelativeTo(null);
        f.requestFocus();
        f.pack();
        m.start();
    }

    private synchronized void start() {
        if(!running) {
            thread = new Thread(this);
            thread.start();
            running = true;
        }
    }

    private synchronized void stop() {
        if(running) {
            try {
                thread.join();
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
            running = false;
        }
    }

    private void init() {
        createBufferStrategy(3);

        ArrayList<Vec2D> waypoints = new ArrayList<>();
        waypoints.add(new Vec2D(200, 600));
        waypoints.add(new Vec2D(300, 200));
        waypoints.add(new Vec2D(600, 300));
        waypoints.add(new Vec2D(800, 600));
        waypoints.add(new Vec2D(900, 350));
        waypoints.add(new Vec2D(1100, 100));
        path = new Path(waypoints);

        for(int i = 0; i < 1000; i++) {
            double maxSpeed = rand.nextDouble() * 90 + 10;
            Vec2D initialVel = Vec2D.randomUnitVec(rand).scale(rand.nextDouble() * maxSpeed);
            double mass = rand.nextDouble() * 0.9 + 0.1;
            double momentOfInertia = rand.nextDouble() * 0.9 + 0.1;
            Vec2D pos = new Vec2D(rand.nextDouble() * WIDTH, rand.nextDouble() * HEIGHT);
            vehicles.add(new Vehicle(pos, 0.0, initialVel.angle(), maxSpeed, mass * 100, mass, momentOfInertia, rand.nextBoolean()));
        }
    }

    @Override
    public void run() {
        init();
        long lastTime = System.nanoTime(), now, delta = 0;
        while(running) {
            now = System.nanoTime();
            delta += now - lastTime;
            if(delta >= NANOSECONDS_PER_TICK) {
                tick((double) NANOSECONDS_PER_TICK / 1000000000.0);
                delta -= NANOSECONDS_PER_TICK;
            }
            render();
            lastTime = now;
        }
        stop();
    }

    private void render() {
        BufferStrategy bs = this.getBufferStrategy();
        Graphics g = bs.getDrawGraphics();
        g.setColor(BACKGROUND_COLOR);
        g.fillRect(0, 0, WIDTH, HEIGHT);

        path.render(g);
        for(Vehicle v : vehicles) v.render(g);

        g.dispose();
        bs.show();
    }

    private void tick(double dt) {
        for(Vehicle v : vehicles) {
            //v.seek(new Vec2D(600, 300));
            //v.arrive(new Vec2D(600, 300), 200);
            v.follow(path, v.maxSpeed, Math.PI, dt);

            v.integrate(dt);
        }
    }
}
