use crate::math::Vec2D;
use crate::math;

pub struct Path {
    waypoints: Vec<Vec2D>,
    wp_dists: Vec<f32>,
}

impl Path {
    fn new(waypoints: Vec<Vec2D>) -> Path {
        let mut wp_dists = Vec::with_capacity(waypoints.len() - 1);
        for i in 0..(waypoints.len() - 1) {
            wp_dists.push(waypoints[i].dist(waypoints[i + 1]));
        }

        Path {
            waypoints,
            wp_dists,
        }
    }

    fn final_waypoint(&self) -> Vec2D {
        self.waypoints[self.waypoints.len() - 1]
    }

    fn point_on_path(&self, mut path_frac: f32) -> Vec2D {
        path_frac = math::clamp_f32(path_frac, 0.0, self.waypoints.len() as f32 - 1.0);
        if path_frac == self.waypoints.len() as f32 - 1.0 {
            return self.final_waypoint();
        }

        let path_int = path_frac as usize;
        let frac = path_frac - path_int as f32;
        Vec2D::lerp(frac, self.waypoints[path_int], self.waypoints[path_int + 1])
    }

    /// Takes a point on the path and slides it some distance along the path.
    ///
    /// `path_frac` - specifies a point on the path
    ///
    /// `slide_dist` - distance to slide
    fn slide(&self, mut path_frac: f32, mut slide_dist: f32) -> Vec2D {
        // Clamp path_frac within appropriate bounds.
        path_frac = math::clamp_f32(path_frac, 0.0, self.waypoints.len() as f32 - 1.0);
        if path_frac == self.waypoints.len() as f32 - 1.0 {
            path_frac -= 1.0;
            slide_dist += self.wp_dists[path_frac as usize];
        }

        // Move to the nearest waypoint behind the specified point.
        let mut path_int = path_frac as usize;
        slide_dist += self.point_on_path(path_frac).dist(self.waypoints[path_int]);

        // Move backwards along the waypoints until the slide distance becomes positive.
        while slide_dist < 0.0 && path_int > 0 {
            path_int -= 1;
            slide_dist += self.wp_dists[path_int];
        }

        if slide_dist < 0.0 {
            return self.waypoints[0];
        }

        // Move forwards along the waypoints until the distance to the next waypoint exceeds the slide distance.
        while slide_dist > self.wp_dists[path_int] && path_int < self.wp_dists.len() - 1 {
            slide_dist -= self.wp_dists[path_int];
            path_int += 1;
        }

        if slide_dist > self.wp_dists[path_int] {
            return self.final_waypoint();
        }

        Vec2D::lerp(slide_dist / self.wp_dists[path_int], self.waypoints[path_int], self.waypoints[path_int + 1])
    }
}

pub struct Vehicle {
    pos: Vec2D,
    speed: f32,
    orientation: f32,
    max_speed: f32,
    max_force: f32,
    mass: f32,
    moment_of_inertia: f32,
    allow_backwards: bool
}

impl Vehicle {
    pub fn vel(&self) -> Vec2D {
        Vec2D::unit_vec(self.orientation).scale(self.speed)
    }

    pub fn calc_thrust_and_torque(&self, mut force: Vec2D) -> (f32, f32) {
        force = force.truncate(self.max_force);
        let orientation_vec = Vec2D::unit_vec(self.orientation);

        let thrust = force.scal_proj(orientation_vec);
        let mut torque = orientation_vec.wedge(force.vec_rej(orientation_vec));
        if self.allow_backwards && self.speed < 0.0 {
            torque *= -1.0;
        }

        (thrust, torque)
    }

    pub fn seek(&self, target: Vec2D) -> (f32, f32) {
        let desired_vel = (target - self.pos).norm().scale(self.max_speed);
        self.calc_thrust_and_torque(desired_vel - self.vel())
    }

    pub fn arrive(&self, dest: Vec2D, stopping_dist: f32) -> (f32, f32) {
        let dist = dest.dist(self.pos);
        if dist >= stopping_dist {
            self.seek(dest)
        } else {
            let desired_vel = (dest - self.pos).norm().scale(self.max_speed * dist / stopping_dist);
            self.calc_thrust_and_torque(desired_vel - self.vel())
        }
    }

    pub fn follow(&self, path: &Path) -> (f32, f32) {
        let future_pos = self.pos + self.vel();

        // If close enough to the end, slow down and arrive at the end.
        if future_pos.dist(path.final_waypoint()) < self.max_speed {
            return self.arrive(path.final_waypoint(), self.max_speed);
        }

        // Otherwise, project the future position onto the nearest point
        // on the path, move that point forward along the path, and follow it.
        let mut to_follow = path.slide(0.0, self.max_speed);
        let mut min_dist = future_pos.dist(path.waypoints[0]);

        for i in 1..path.waypoints.len() {
            let waypoint = path.waypoints[i];
            let dist = future_pos.dist(waypoint);
            if dist < min_dist {
                min_dist = dist;
                to_follow = path.slide(i as f32, self.max_speed);
            }
        }

        for i in 0..(path.waypoints.len() - 1) {
            let w0 = path.waypoints[i];
            let w1 = path.waypoints[i + 1];
            let path_segment = w1 - w0;
            let proj_future_pos = (future_pos - w0).vec_proj(path_segment) + w0;
            let frac = Vec2D::inv_lerp(proj_future_pos, w0, w1);
            let dist = future_pos.dist(proj_future_pos);
            if frac > 0.0 && frac < 1.0 && dist < min_dist {
                min_dist = dist;
                to_follow = path.slide(i as f32 + frac, self.max_speed);
            }
        }

        self.seek(to_follow)
    }
}
