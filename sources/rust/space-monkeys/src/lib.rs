use std::f32::consts;
use std::fs::File;
use std::io;
use std::io::{ErrorKind, BufReader, BufRead};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt};

const INTERACTION_RADIUS: f32 = 0.2;
const INTERACTION_RADIUS_SQR: f32 = INTERACTION_RADIUS * INTERACTION_RADIUS;

#[derive(Clone, Debug, PartialEq)]
struct Vec2D {
    x: f32,
    y: f32,
}

impl Vec2D {
    fn add(&self, rhs: &Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }

    fn add_mut(&mut self, rhs: &Self) -> &mut Self {
        self.x += rhs.x;
        self.y += rhs.y;
        self
    }

    fn sub(&self, rhs: &Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }

    fn sub_mut(&mut self, rhs: &Self) -> &mut Self {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self
    }

    fn scale(&self, factor: f32) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }

    fn scale_mut(&mut self, factor: f32) -> &mut Self {
        self.x *= factor;
        self.y *= factor;
        self
    }

    fn l2_sqr(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    fn wedge(&self, rhs: &Self) -> f32 {
        self.x * rhs.y - self.y * rhs.x
    }
}

#[derive(Clone, Debug, PartialEq)]
struct Vec3D {
    x: f32,
    y: f32,
    z: f32,
}

impl Vec3D {
    fn add(&self, rhs: &Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }

    fn add_mut(&mut self, rhs: &Self) -> &mut Self {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
        self
    }

    fn sub(&self, rhs: &Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }

    fn sub_mut(&mut self, rhs: &Self) -> &mut Self {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
        self
    }

    fn scale(&self, factor: f32) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }

    fn scale_mut(&mut self, factor: f32) -> &mut Self {
        self.x *= factor;
        self.y *= factor;
        self.z *= factor;
        self
    }

    fn l2_sqr(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    fn cross(&self, rhs: &Self) -> Self {
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

#[derive(Debug)]
struct Tri {
    v1: u32,
    v2: u32,
    v3: u32,
}

/// Read mesh data from a binary PLY file.
///
/// Expected file format:
/// ```
/// ----Header----
/// "ply\n" - ASCII
/// "format binary_little_endian 1.0\n" - ASCII
/// "element vertex ####\n" - ASCII
/// "property float32 x\n" - ASCII
/// "property float32 y\n" - ASCII
/// "property float32 z\n" - ASCII
/// "element face ####\n" - ASCII
/// "property list uchar int vertex_indices\n" - ASCII
/// "end_header\n" - ASCII
/// ----Data (variable size)----
/// ```
/// Formatted according to header - binary
fn read_mesh_from_file(filename: &str) -> io::Result<(Vec<Vec3D>, Vec<Tri>)> {
    println!("Reading mesh from {}...", filename);
    let file = File::open(&Path::new(filename))?;
    let mut reader = BufReader::new(file);
    let mut line = String::new();

    // Read header
    read_line(&mut reader, &mut line)?;
    if !line.eq("ply") {
        return Err(io::Error::new(ErrorKind::InvalidData, "not a PLY file"));
    }

    read_line(&mut reader, &mut line)?;
    if !line.eq("format binary_little_endian 1.0") {
        return Err(io::Error::new(ErrorKind::InvalidData, "not formatted in binary little-endian 1.0"));
    }

    // Skip the comment
    read_line(&mut reader, &mut line)?;

    read_line(&mut reader, &mut line)?;
    let next_line = line.to_owned();
    let next_line: Vec<&str> = next_line.split(' ').collect();
    if !next_line.get(0).unwrap_or(&"").eq(&"element") || !next_line.get(1).unwrap_or(&"").eq(&"vertex") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    let num_vertices = next_line.get(2).unwrap_or(&"").parse::<u32>()
        .map_err(|err| io::Error::new(ErrorKind::InvalidData, format!("could not parse number of vertices: {}", err)))?;

    let mut vertices = Vec::with_capacity(num_vertices as usize);

    read_line(&mut reader, &mut line)?;
    if !line.eq("property float32 x") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    read_line(&mut reader, &mut line)?;
    if !line.eq("property float32 y") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    read_line(&mut reader, &mut line)?;
    if !line.eq("property float32 z") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    read_line(&mut reader, &mut line)?;
    let next_line = line.to_owned();
    let next_line: Vec<&str> = next_line.split(' ').collect();
    if !next_line.get(0).unwrap_or(&"").eq(&"element") || !next_line.get(1).unwrap_or(&"").eq(&"face") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    let num_tris = next_line.get(2).unwrap_or(&"").parse::<u32>()
        .map_err(|err| io::Error::new(ErrorKind::InvalidData, format!("could not parse number of tris: {}", err)))?;

    let mut tris = Vec::with_capacity(num_tris as usize);

    read_line(&mut reader, &mut line)?;
    if !line.eq("property list uchar int vertex_indices") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    read_line(&mut reader, &mut line)?;
    if !line.eq("end_header") {
        return Err(io::Error::new(ErrorKind::InvalidData, "end of header not found"));
    }

    // Read binary data
    for _ in 0..num_vertices {
        let x = reader.read_f32::<LittleEndian>()?;
        let y = reader.read_f32::<LittleEndian>()?;
        let z = reader.read_f32::<LittleEndian>()?;
        vertices.push(Vec3D { x, y, z });
    }

    for _ in 0..num_tris {
        if reader.read_u8()? != 3 {
            return Err(io::Error::new(ErrorKind::InvalidData, "found a non-tri polygon"));
        }
        let v1 = reader.read_u32::<LittleEndian>()?;
        let v2 = reader.read_u32::<LittleEndian>()?;
        let v3 = reader.read_u32::<LittleEndian>()?;
        tris.push(Tri { v1, v2, v3 });
    }

    Ok((vertices, tris))
}

fn read_line(reader: &mut BufReader<File>, line: &mut String) -> io::Result<usize> {
    line.clear();
    let result = reader.read_line(line);
    if result.is_ok() {
        line.pop();
    }
    result
}

fn filter_mesh(vertices: &mut Vec<Vec3D>, tris: &mut Vec<Tri>) {
    println!("Removing upside-down tris...");
    tris.retain(|tri| -> bool {
        let t1 = vertices[tri.v2 as usize].sub(&vertices[tri.v1 as usize]);
        let t2 = vertices[tri.v3 as usize].sub(&vertices[tri.v1 as usize]);
        t1.cross(&t2).y >= 0.0
    });

    println!("Removing tris far above the camera...");
    tris.retain(|tri| -> bool {
        let t1 = &vertices[tri.v1 as usize];
        let t2 = &vertices[tri.v2 as usize];
        let t3 = &vertices[tri.v3 as usize];
        t1.y < 1.0 || t2.y < 1.0 || t3.y < 1.0
    });
}

struct SmoothedParticle {
    pos: Vec2D,
    height: f32,
    area: f32,
}

fn to_particles(vertices: &[Vec3D], tris: &[Tri]) -> Vec<SmoothedParticle> {
    let mut particles = Vec::new();

    for tri in tris {
        let t1 = Vec2D { x: vertices[tri.v1 as usize].x, y: vertices[tri.v1 as usize].z };
        let t2 = Vec2D { x: vertices[tri.v2 as usize].x, y: vertices[tri.v2 as usize].z };
        let t3 = Vec2D { x: vertices[tri.v3 as usize].x, y: vertices[tri.v3 as usize].z };

        // The centroid
        let pos = t1.add(&t2).add(&t3).scale(1.0 / 3.0);
        let height = (vertices[tri.v1 as usize].y + vertices[tri.v2 as usize].y + vertices[tri.v3 as usize].y) / 3.0;
        let area = t2.sub(&t1).wedge(&t3.sub(&t1)).abs() / 2.0;

        if height <= 1.0 {
            particles.push(SmoothedParticle { pos, height, area });
        }
    }

    particles
}

fn terrain_height(particles: &[SmoothedParticle], p: Vec2D) -> f32 {
    let mut height = 0.0;
    for particle in particles {
        let d_sqr = p.sub(&particle.pos).l2_sqr();
        if d_sqr < INTERACTION_RADIUS_SQR {
            height += particle.area * particle.height * (4.0 / (consts::PI * INTERACTION_RADIUS_SQR.powf(4.0))) * (INTERACTION_RADIUS_SQR - d_sqr).powf(3.0);
        }
    }
    height
}

fn terrain_gradient(particles: &[SmoothedParticle], p: Vec2D) -> f32 {
    let d = 0.001;
    let grad_x = (terrain_height(particles, p.add(&Vec2D { x: d, y: 0.0 })) - terrain_height(particles, p.sub(&Vec2D { x: d, y: 0.0 }))) / (2.0 * d);
    let grad_y = (terrain_height(particles, p.add(&Vec2D { x: 0.0, y: d })) - terrain_height(particles, p.sub(&Vec2D { x: 0.0, y: d }))) / (2.0 * d);
    (grad_x * grad_x + grad_y * grad_y).sqrt()
}

pub fn mesh_to_grid(filename: &str, min_x: f32, max_x: f32, min_z: f32, max_z: f32, res_x: usize,
                    res_z: usize) -> io::Result<Grid> {
    let (mut vertices, mut tris) = read_mesh_from_file(filename)?;
    filter_mesh(&mut vertices, &mut tris);

    let particles = to_particles(&vertices, &tris);

    println!("Generating terrain grid...");
    let mut grid = Grid::new(min_x, max_x, min_z, max_z, res_x, res_z);
    for grid_z in 0..res_z {
        for grid_x in 0..res_x {
            let x = lerp_f32(min_x, max_x, grid_x as f32 / res_x as f32);
            let z = lerp_f32(min_z, max_z, grid_z as f32 / res_z as f32);
            grid.set_height(grid_x, grid_z, terrain_height(&particles, Vec2D { x, y: z }));
            grid.set_grad_mag(grid_x, grid_z, terrain_gradient(&particles, Vec2D { x, y: z }));
        }
    }
    grid.update_extrema();

    Ok(grid)
}

pub struct Grid {
    min_x: f32,
    max_x: f32,
    min_height: f32,
    max_height: f32,
    min_grad_mag: f32,
    max_grad_mag: f32,
    min_z: f32,
    max_z: f32,
    res_x: usize,
    res_z: usize,
    height: Vec<f32>,
    grad_mag: Vec<f32>,
}

impl Grid {
    fn new(min_x: f32, max_x: f32, min_z: f32, max_z: f32, res_x: usize, res_z: usize) -> Grid {
        Grid {
            min_x,
            max_x,
            min_height: 0.0,
            max_height: 0.0,
            min_grad_mag: 0.0,
            max_grad_mag: 0.0,
            min_z,
            max_z,
            res_x,
            res_z,
            height: vec![0.0; res_x * res_z],
            grad_mag: vec![0.0; res_x * res_z],
        }
    }

    /// The height at a grid cell.
    pub fn get_height(&self, x: usize, z: usize) -> f32 {
        self.height[z * self.res_x + x]
    }

    /// The height at a grid cell, normalized between 0 and 1.
    pub fn get_height_normed(&self, x: usize, z: usize) -> f32 {
        if (self.min_height - self.max_height).abs() < f32::EPSILON {
            0.0
        } else {
            (self.height[z * self.res_x + x] - self.min_height) / (self.max_height - self.min_height)
        }
    }

    fn set_height(&mut self, x: usize, z: usize, value: f32) {
        self.height[z * self.res_x + x] = value;
    }

    /// The magnitude of the gradient at a grid cell.
    pub fn get_grad_mag(&self, x: usize, z: usize) -> f32 {
        self.grad_mag[z * self.res_x + x]
    }

    /// The magnitude of the gradient at a grid cell, normalized between 0 and 1.
    pub fn get_grad_mag_normed(&self, x: usize, z: usize) -> f32 {
        if (self.min_grad_mag - self.max_grad_mag).abs() < f32::EPSILON {
            0.0
        } else {
            (self.grad_mag[z * self.res_x + x] - self.min_grad_mag) / (self.max_grad_mag - self.min_grad_mag)
        }
    }

    fn set_grad_mag(&mut self, x: usize, z: usize, value: f32) {
        self.grad_mag[z * self.res_x + x] = value;
    }

    fn update_extrema(&mut self) {
        self.min_height = self.height.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        self.max_height = self.height.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        self.min_grad_mag = self.grad_mag.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        self.max_grad_mag = self.grad_mag.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use plotters::drawing::IntoDrawingArea;
    use plotters::prelude::BitMapBackend;
    use plotters::style::{BLACK, HSLColor};

    use crate::{lerp_f32, mesh_to_grid, Vec2D, Vec3D};

    #[test]
    fn test_mesh() {
        let min_x = -5.0;
        let max_x = 1.0;
        let min_z = -7.0;
        let max_z = 0.0;
        let res_x = 128;
        let res_z = 128;

        let start = Instant::now();
        let grid = mesh_to_grid("../test-data/test_mesh.ply", min_x, max_x, min_z, max_z, res_x, res_z);
        let elapsed = start.elapsed();

        match grid {
            Ok(grid) => {
                let terrain_height = BitMapBackend::new("../images/terrain_height.png", (res_x as u32, res_z as u32)).into_drawing_area();
                let terrain_grad_mag = BitMapBackend::new("../images/terrain_grad_mag.png", (res_x as u32, res_z as u32)).into_drawing_area();
                terrain_height.fill(&BLACK).unwrap();
                terrain_grad_mag.fill(&BLACK).unwrap();

                for grid_z in 0..res_z {
                    for grid_x in 0..res_x {
                        let height = grid.get_height_normed(grid_x, grid_z);
                        let grad_mag = grid.get_grad_mag_normed(grid_x, grid_z);

                        let colorizer = |value: f32| -> HSLColor {
                            let mut hue = lerp_f32(240.0 / 360.0, -60.0 / 360.0, value) as f64;
                            while hue < 0.0 { hue += 1.0; }
                            let sat = (value * value - value) as f64 + 1.0;
                            let lit = (2.0 * value as f64 - 1.0).powf(3.0) / 2.0 + 0.5;
                            return HSLColor(hue, sat, lit);
                        };

                        terrain_height.draw_pixel((grid_x as i32, grid_z as i32), &colorizer(height)).unwrap();
                        terrain_grad_mag.draw_pixel((grid_x as i32, grid_z as i32), &colorizer(grad_mag)).unwrap();
                    }
                }
            }
            Err(err) => panic!("{}", err)
        }
        println!("Elapsed: {:?}", elapsed);
        assert!(elapsed < Duration::from_secs(1));
    }

    #[test]
    fn test_vec2d() {
        let a = Vec2D { x: 3.0, y: 2.0 };
        let b = Vec2D { x: -1.0, y: 4.0 };
        assert_eq!(a.add(&b), Vec2D { x: 2.0, y: 6.0 });
        assert_eq!(a.sub(&b), Vec2D { x: 4.0, y: -2.0 });
        assert_eq!(a.scale(3.0), Vec2D { x: 9.0, y: 6.0 });
        assert_eq!(Vec2D { x: 3.0, y: 2.0 }.add_mut(&b).to_owned(), Vec2D { x: 2.0, y: 6.0 });
        assert_eq!(Vec2D { x: 3.0, y: 2.0 }.sub_mut(&b).to_owned(), Vec2D { x: 4.0, y: -2.0 });
        assert_eq!(Vec2D { x: 3.0, y: 2.0 }.scale_mut(3.0).to_owned(), Vec2D { x: 9.0, y: 6.0 });
        assert_eq!(a.l2_sqr(), 13.0);
        assert_eq!(a.wedge(&b), 14.0);
    }

    #[test]
    fn test_vec3d() {
        let a = Vec3D { x: 3.0, y: 2.0, z: -5.0 };
        let b = Vec3D { x: -1.0, y: 4.0, z: 1.0 };
        assert_eq!(a.add(&b), Vec3D { x: 2.0, y: 6.0, z: -4.0 });
        assert_eq!(a.sub(&b), Vec3D { x: 4.0, y: -2.0, z: -6.0 });
        assert_eq!(a.scale(3.0), Vec3D { x: 9.0, y: 6.0, z: -15.0 });
        assert_eq!(Vec3D { x: 3.0, y: 2.0, z: -5.0 }.add_mut(&b).to_owned(), Vec3D { x: 2.0, y: 6.0, z: -4.0 });
        assert_eq!(Vec3D { x: 3.0, y: 2.0, z: -5.0 }.sub_mut(&b).to_owned(), Vec3D { x: 4.0, y: -2.0, z: -6.0 });
        assert_eq!(Vec3D { x: 3.0, y: 2.0, z: -5.0 }.scale_mut(3.0).to_owned(), Vec3D { x: 9.0, y: 6.0, z: -15.0 });
        assert_eq!(a.l2_sqr(), 38.0);
        assert_eq!(a.cross(&b), Vec3D { x: 22.0, y: 2.0, z: 14.0 });
    }
}
