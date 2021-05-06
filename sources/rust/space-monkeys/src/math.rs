use std::f32;
use std::ops::{Add, Sub};

pub const ZERO_2D: Vec2D = Vec2D { x: 0.0, y: 0.0 };
pub const ZERO_3D: Vec3D = Vec3D { x: 0.0, y: 0.0, z: 0.0 };

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2D {
    pub x: f32,
    pub y: f32,
}

impl Vec2D {
    pub fn unit_vec(angle: f32) -> Self {
        Vec2D {
            x: angle.cos(),
            y: angle.sin(),
        }
    }

    pub fn lerp(frac: f32, low: Vec2D, high: Vec2D) -> Self {
        low.scale(1.0 - frac) + high.scale(frac)
    }

    pub fn inv_lerp(lerped: Vec2D, low: Vec2D, high: Vec2D) -> f32 {
        let frac_x = (lerped.x - low.x) / (high.x - low.x);
        let frac_y = (lerped.y - low.y) / (high.y - low.y);
        if high.x == low.x {
            return frac_y;
        }
        if high.y == low.y {
            return frac_x;
        }
        if (frac_x - frac_y).abs() > 1e-8 {
            panic!("Arguments are not collinear")
        }
        (frac_x + frac_y) / 2.0
    }

    pub fn scale(&self, factor: f32) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }

    pub fn l2_sqr(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    pub fn l2(&self) -> f32 {
        self.l2_sqr().sqrt()
    }

    pub fn dist_sqr(self, other: Vec2D) -> f32 {
        (other - self).l2_sqr()
    }

    pub fn dist(self, other: Vec2D) -> f32 {
        self.dist_sqr(other).sqrt()
    }

    pub fn angle(&self) -> f32 {
        let mut angle = self.y.atan2(self.x);
        while angle < 0.0 {
            angle += 2.0 * f32::consts::PI;
        }
        angle
    }

    pub fn rot(self, angle: f32) -> Self {
        Vec2D {
            x: self.x * angle.cos() - self.y * angle.sin(),
            y: self.x * angle.sin() + self.y * angle.cos(),
        }
    }

    pub fn truncate(self, max_length: f32) -> Vec2D {
        self.norm().scale(self.l2().min(max_length))
    }

    pub fn dot(self, other: Vec2D) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn wedge(&self, rhs: Vec2D) -> f32 {
        self.x * rhs.y - self.y * rhs.x
    }

    pub fn perp(self) -> Vec2D {
        Vec2D {
            x: self.y,
            y: -self.x
        }
    }

    pub fn norm(self) -> Vec2D {
        if self == ZERO_2D {
            ZERO_2D
        } else {
            self.scale(1.0 / self.l2())
        }
    }

    pub fn scal_proj(self, onto: Vec2D) -> f32 {
        self.dot(onto.norm())
    }

    pub fn vec_proj(self, onto: Vec2D) -> Vec2D {
        onto.scale(self.dot(onto) / onto.dot(onto))
    }

    pub fn vec_rej(self, from: Vec2D) -> Vec2D {
        self.sub(self.vec_proj(from))
    }
}

impl Add for Vec2D {
    type Output = Vec2D;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Vec2D {
    type Output = Vec2D;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Vec3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3D {
    pub fn add(&self, rhs: &Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }

    pub fn add_mut(&mut self, rhs: &Self) -> &mut Self {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
        self
    }

    pub fn sub(&self, rhs: &Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }

    pub fn sub_mut(&mut self, rhs: &Self) -> &mut Self {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
        self
    }

    pub fn scale(&self, factor: f32) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }

    pub fn scale_mut(&mut self, factor: f32) -> &mut Self {
        self.x *= factor;
        self.y *= factor;
        self.z *= factor;
        self
    }

    pub fn l2_sqr(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn cross(&self, rhs: &Self) -> Self {
        Vec3D {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}

pub fn lerp_f32(low: f32, high: f32, frac: f32) -> f32 {
    low * (1.0 - frac) + high * frac
}

pub fn clamp_f32(value: f32, min: f32, max: f32) -> f32 {
    if min > max {
        panic!("min must be less than or equal to max")
    }
    value.max(min).min(max)
}
