package main;

import java.awt.*;

public class Vehicle {

    private static final double RENDER_RADIUS = 10;
    private static final Color COLOR = Color.GREEN;

    private Vec2D pos;
    private double speed;
    private Tuple<Double, Double> thrustAndTorque;
    private double orientation;
    public  final double maxSpeed;
    private final double maxForce;
    private final double mass;
    private final double momentOfInertia;
    private final boolean allowBackwards;

    // Just for visuals
    private Vec2D target;

    public Vehicle(Vec2D pos, double speed, double orientation, double maxSpeed, double maxForce, double mass, double momentOfInertia, boolean allowBackwards) {
        this.pos = pos;
        this.speed = speed;
        thrustAndTorque = new Tuple<>(0.0, 0.0);
        this.orientation = orientation;
        this.maxSpeed = maxSpeed;
        this.maxForce = maxForce;
        this.mass = mass;
        this.momentOfInertia = momentOfInertia;
        this.allowBackwards = allowBackwards;

        // Just for visuals
        this.target = null;
    }

    private Vec2D vel() {
        return Vec2D.unitVec(orientation).scale(speed);
    }

    public void render(Graphics g) {
        g.setColor(COLOR);
        g.drawOval((int) (pos.x - RENDER_RADIUS), (int) (pos.y - RENDER_RADIUS), (int) (2 * RENDER_RADIUS), (int) (2 * RENDER_RADIUS));
        Vec2D ori = Vec2D.unitVec(orientation).scale(RENDER_RADIUS);
        g.drawLine((int) pos.x, (int) pos.y, (int) (pos.x + ori.x), (int) (pos.y + ori.y));

        g.setColor(Color.RED);
        g.drawLine((int) pos.x, (int) pos.y, (int) (pos.x + vel().x), (int) (pos.y + vel().y));

        if(target != null) {
            g.setColor(Color.WHITE);
            //g.drawOval((int) (target.x - RENDER_RADIUS), (int) (target.y - RENDER_RADIUS), (int) (2 * RENDER_RADIUS), (int) (2 * RENDER_RADIUS));
        }
    }

    public Tuple<Double, Double> calcThrustAndTorque(Vec2D force) {
        force = force.truncate(maxForce);
        Vec2D orientationVec = Vec2D.unitVec(orientation);

        double thrust = force.scalProj(orientationVec);
        double torque = orientationVec.cross(force.vecRej(orientationVec));
        if(allowBackwards && speed < 0.0) torque *= -1;

        return new Tuple<>(thrust, torque);
    }

    public void integrate(double dt) {
        // This integration will be determined by the physical robot.
        speed += thrustAndTorque.a / mass * dt;
        speed = Util.clamp(speed, -maxSpeed, maxSpeed);

        orientation += thrustAndTorque.b / momentOfInertia * dt * dt;
        pos = pos.add(vel().scale(dt));
    }

    // ---- BEHAVIORS ---- //
    public void seek(Vec2D target) {
        Vec2D desiredVel = target.sub(pos).norm().scale(maxSpeed);
        Vec2D force = desiredVel.sub(vel());
        thrustAndTorque = calcThrustAndTorque(force);

        // Just for visuals
        this.target = target;
    }

    public void arrive(Vec2D dest, double stoppingDist) {
        double dist = dest.dist(pos);
        if(dist >= stoppingDist) {
            seek(dest);
        } else {
            Vec2D desiredVel = dest.sub(pos).norm().scale(maxSpeed * dist / stoppingDist);
            Vec2D force = desiredVel.sub(vel());
            thrustAndTorque = calcThrustAndTorque(force);

            // Just for visuals
            target = dest;
        }
    }

    public void arrive(Vec2D dest, double stoppingDist, double targetOrientation) {
        Vec2D disp = dest.sub(pos);
        double dist = disp.length();
        if(dist >= stoppingDist) {
            seek(dest);
        } else {
            Vec2D desiredVel = disp.norm().scale(maxSpeed * dist / stoppingDist);
            Vec2D force = desiredVel.sub(vel());
            Tuple<Double, Double> thrustAndTorque = calcThrustAndTorque(force);

            Vec2D dispRot = disp.rot(-targetOrientation);
            double desiredOrientation = new Vec2D(dispRot.x * dispRot.x - dispRot.y * dispRot.y + maxSpeed, 2 * dispRot.x * dispRot.y).rot(targetOrientation).angle();

            double orientationDiff = desiredOrientation - orientation;
            while(orientationDiff > Math.PI) orientationDiff -= 2 * Math.PI;
            while(orientationDiff < -Math.PI) orientationDiff += 2 * Math.PI;

            this.thrustAndTorque = new Tuple<>(thrustAndTorque.a, thrustAndTorque.b + maxSpeed * (1 - dist / stoppingDist) * orientationDiff);

            // Just for visuals
            target = dest;
        }
    }

    public void follow(Path path, double stoppingDist, double targetOrientation, double dt) {
        // If close enough to the end, slow down and arrive at the end.
        if(pos.dist(path.finalWaypoint()) < stoppingDist) {
            arrive(path.finalWaypoint(), stoppingDist, targetOrientation);
            return;
        }

        // Otherwise, project the future position onto the nearest point
        // on the path, move that point forward along the path, and follow it.
        Vec2D futurePos = pos.add(vel().scale(dt));
        Vec2D toFollow = path.slide(0, maxSpeed);
        double minDist = futurePos.dist(path.waypoints.get(0));

        for(int i = 1; i < path.waypoints.size(); i++) {
            Vec2D waypoint = path.waypoints.get(i);
            double dist = futurePos.dist(waypoint);
            if(dist < minDist) {
                minDist = dist;
                toFollow = path.slide(i, maxSpeed);
            }
        }

        for(int i = 0; i < path.waypoints.size() - 1; i++) {
            Vec2D w0 = path.waypoints.get(i);
            Vec2D w1 = path.waypoints.get(i + 1);
            Vec2D pathSegment = w1.sub(w0);
            Vec2D projFuturePos = futurePos.sub(w0).vecProj(pathSegment).add(w0);
            double frac = Vec2D.invLerp(projFuturePos, w0, w1);
            double dist = futurePos.dist(projFuturePos);
            if(frac > 0.0 && frac < 1.0 && dist < minDist) {
                minDist = dist;
                toFollow = path.slide(i + frac, maxSpeed);
            }
        }

        seek(toFollow);
    }
}
