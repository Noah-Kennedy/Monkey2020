package main;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class Path {

    private static final Color COLOR = Color.WHITE;

    /**The global position of each waypoint.*/
    public final List<Vec2D> waypoints;

    /**The distance between each consecutive waypoint.*/
    private final List<Double> wpDists;

    public Path(List<Vec2D> waypoints) {
        this.waypoints = waypoints;

        wpDists = new ArrayList<>();
        for(int i = 0; i < waypoints.size() - 1; i++) {
            wpDists.add(waypoints.get(i).dist(waypoints.get(i + 1)));
        }
    }

    public Vec2D finalWaypoint() {
        return waypoints.get(waypoints.size() - 1);
    }

    public Vec2D pointOnPath(double pathFrac) {
        pathFrac = Util.clamp(pathFrac, 0, waypoints.size() - 1);
        if(pathFrac == waypoints.size() - 1) return finalWaypoint();

        int pathInt = (int) pathFrac;
        double frac = pathFrac - pathInt;
        return Vec2D.lerp(frac, waypoints.get(pathInt), waypoints.get(pathInt + 1));
    }

    /**
     * Takes a point on the path and slides it some distance along the path.
     * @param pathFrac specifies a point on the path
     * @param slideDist distance to slide
     * @return The point that is `slideDist` away from the point given by `pathFrac`, by path length.
     */
    public Vec2D slide(double pathFrac, double slideDist) {
        // Clamp pathFrac within appropriate bounds.
        pathFrac = Util.clamp(pathFrac, 0, waypoints.size() - 1);
        if(pathFrac == waypoints.size() - 1) {
            pathFrac -= 1;
            slideDist += wpDists.get((int) pathFrac);
        }

        // Move to the nearest waypoint behind the specified point.
        int pathInt = (int) pathFrac;
        slideDist += pointOnPath(pathFrac).dist(waypoints.get(pathInt));

        // Move backwards along the waypoints until the slide distance becomes positive.
        while(slideDist < 0 && pathInt > 0) {
            pathInt--;
            slideDist += wpDists.get(pathInt);
        }

        if(slideDist < 0) {
            return waypoints.get(0);
        }

        // Move forwards along the waypoints until the distance to the next waypoint exceeds the slide distance.
        while(slideDist > wpDists.get(pathInt) && pathInt < wpDists.size() - 1) {
            slideDist -= wpDists.get(pathInt);
            pathInt++;
        }

        if(slideDist > wpDists.get(pathInt)) {
            return finalWaypoint();
        }

        return Vec2D.lerp(slideDist / wpDists.get(pathInt), waypoints.get(pathInt), waypoints.get(pathInt + 1));
    }

    public void render(Graphics g) {
        g.setColor(COLOR);
        for(int i = 0; i < waypoints.size() - 1; i++) {
            Vec2D w0 = waypoints.get(i);
            Vec2D w1 = waypoints.get(i + 1);
            g.drawLine((int) w0.x, (int) w0.y, (int) w1.x, (int) w1.y);
        }
    }
}
