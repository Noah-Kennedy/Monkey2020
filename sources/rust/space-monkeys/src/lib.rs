use std::path::Path;
use std::fs::File;
use std::io;
use std::io::ErrorKind;
use std::f32;
use byteorder::{ReadBytesExt, LittleEndian};

const INTERACTION_RADIUS: f32 = 0.20;
const INTERACTION_RADIUS_SQR: f32 = INTERACTION_RADIUS * INTERACTION_RADIUS;

#[derive(Clone, Debug, PartialEq)]
struct Vec2D {
    x: f32,
    y: f32
}

impl Vec2D {
    fn add(&self, rhs: &Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y
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
            y: self.y - rhs.y
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
            y: self.y * factor
        }
    }

    fn scale_mut(&mut self, factor: f32) -> &mut Self {
        self.x *= factor;
        self.y *= factor;
        self
    }

    fn l2(&self) -> f32 {
        self.l2_sqr().sqrt()
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
    z: f32
}

impl Vec3D {
    fn add(&self, rhs: &Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z
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
            z: self.z - rhs.z
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
            z: self.z * factor
        }
    }

    fn scale_mut(&mut self, factor: f32) -> &mut Self {
        self.x *= factor;
        self.y *= factor;
        self.z *= factor;
        self
    }

    fn l2(&self) -> f32 {
        self.l2_sqr().sqrt()
    }

    fn l2_sqr(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    fn cross(&self, rhs: &Self) -> Self {
        Vec3D {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x
        }
    }
}

fn lerp_f32(low: f32, high: f32, frac: f32) -> f32 {
    low * (1.0 - frac) + high * frac
}

fn lerp_vec2d(low: &Vec2D, high: &Vec2D, frac: f32) -> Vec2D {
    Vec2D {
        x: low.x * (1.0 - frac) + high.x * frac,
        y: low.y * (1.0 - frac) + high.y * frac
    }
}

fn lerp_vec3d(low: &Vec3D, high: &Vec3D, frac: f32) -> Vec3D {
    Vec3D {
        x: low.x * (1.0 - frac) + high.x * frac,
        y: low.y * (1.0 - frac) + high.y * frac,
        z: low.z * (1.0 - frac) + high.z * frac
    }
}

#[derive(Debug)]
struct Tri {
    v1: u32,
    v2: u32,
    v3: u32
}

/// Read mesh data from a binary PLY file.
///
/// Expected file format:
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
/// Formatted according to header - binary
fn read_mesh_from_file(filename: &str) -> io::Result<(Vec<Vec3D>, Vec<Tri>)> {
    println!("Reading mesh from {}...", filename);
    let mut file = File::open(&Path::new(filename))?;

    // Read header
    if !read_line(&mut file)?.eq("ply") {
        return Err(io::Error::new(ErrorKind::InvalidData, "not a PLY file"));
    }

    if !read_line(&mut file)?.eq("format binary_little_endian 1.0") {
        return Err(io::Error::new(ErrorKind::InvalidData, "not formatted in binary little-endian 1.0"));
    }

    // Skip the comment
    read_line(&mut file)?;

    let next_line = read_line(&mut file)?;
    let next_line: Vec<&str> = next_line.split(' ').collect();
    if !next_line.get(0).unwrap_or(&"").eq(&"element") || !next_line.get(1).unwrap_or(&"").eq(&"vertex") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    let num_vertices = next_line.get(2).unwrap_or(&"").parse::<u32>()
        .map_err(|err| io::Error::new(ErrorKind::InvalidData, format!("could not parse number of vertices: {}", err)))?;

    let mut vertices = Vec::with_capacity(num_vertices as usize);

    if !read_line(&mut file)?.eq("property float32 x") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    if !read_line(&mut file)?.eq("property float32 y") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    if !read_line(&mut file)?.eq("property float32 z") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    let next_line = read_line(&mut file)?;
    let next_line: Vec<&str> = next_line.split(' ').collect();
    if !next_line.get(0).unwrap_or(&"").eq(&"element") || !next_line.get(1).unwrap_or(&"").eq(&"face") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    let num_tris = next_line.get(2).unwrap_or(&"").parse::<u32>()
        .map_err(|err| io::Error::new(ErrorKind::InvalidData, format!("could not parse number of tris: {}", err)))?;

    let mut tris = Vec::with_capacity(num_tris as usize);

    if !read_line(&mut file)?.eq("property list uchar int vertex_indices") {
        return Err(io::Error::new(ErrorKind::InvalidData, "unexpected header strings encountered"));
    }

    if !read_line(&mut file)?.eq("end_header") {
        return Err(io::Error::new(ErrorKind::InvalidData, "end of header not found"));
    }

    // Read binary data
    for _ in 0..num_vertices {
        let x = file.read_f32::<LittleEndian>()?;
        let y = file.read_f32::<LittleEndian>()?;
        let z = file.read_f32::<LittleEndian>()?;
        vertices.push(Vec3D { x, y, z });
    }

    for _ in 0..num_tris {
        if file.read_u8()? != 3 {
            return Err(io::Error::new(ErrorKind::InvalidData, "found a non-tri polygon"));
        }
        let v1 = file.read_u32::<LittleEndian>()?;
        let v2 = file.read_u32::<LittleEndian>()?;
        let v3 = file.read_u32::<LittleEndian>()?;
        tris.push(Tri { v1, v2, v3 });
    }

    Ok((vertices, tris))
}

fn read_line(file: &mut File) -> io::Result<String> {
    let mut s = String::new();
    while !s.ends_with('\n') {
        s.push(file.read_u8()? as char);
    }
    s.pop();
    Ok(s)
}

fn remove_redundant_vertices(vertices: &mut Vec<Vec3D>, tris: &mut Vec<Tri>) {
    for i in (1..vertices.len()).rev() {
        println!("{}", i);
        for j in 0..(i - 1) {
            if vertices[i] == vertices[j] {
                vertices.remove(i);
                for tri in &mut *tris {
                    if tri.v1 == i as u32 {
                        tri.v1 = j as u32;
                    } else if tri.v2 == i as u32 {
                        tri.v2 = j as u32;
                    } else if tri.v3 == i as u32 {
                        tri.v3 = j as u32;
                    }

                    if tri.v1 > i as u32 {
                        tri.v1 -= 1;
                    }
                    if tri.v2 > i as u32 {
                        tri.v2 -= 1;
                    }
                    if tri.v3 > i as u32 {
                        tri.v3 -= 1;
                    }
                }
                break;
            }
        }
    }
}

fn remove_upside_down_tris(vertices: &mut Vec<Vec3D>, tris: &mut Vec<Tri>) {
    tris.retain(|tri| -> bool {
        let t1 = vertices[tri.v2 as usize].sub(&vertices[tri.v1 as usize]);
        let t2 = vertices[tri.v3 as usize].sub(&vertices[tri.v1 as usize]);
        t1.cross(&t2).y >= 0.0
    });
}

fn remove_tris_far_above_camera(vertices: &mut Vec<Vec3D>, tris: &mut Vec<Tri>) {
    tris.retain(|tri| -> bool {
        let t1 = &vertices[tri.v1 as usize];
        let t2 = &vertices[tri.v2 as usize];
        let t3 = &vertices[tri.v3 as usize];
        t1.y < 1.0 || t2.y < 1.0 || t3.y < 1.0
    });
}

fn remove_unused_vertices(vertices: &mut Vec<Vec3D>, tris: &mut Vec<Tri>) {
    for i in (0..vertices.len()).rev() {
        let mut vertex_used = false;
        for tri in tris.iter() {
            if tri.v1 == i as u32 || tri.v2 == i as u32 || tri.v3 == i as u32 {
                vertex_used = true;
                break;
            }
        }

        if !vertex_used {
            vertices.remove(i);
            for tri in &mut *tris {
                if tri.v1 > i as u32 {
                    tri.v1 -= 1;
                }
                if tri.v2 > i as u32 {
                    tri.v2 -= 1;
                }
                if tri.v3 > i as u32 {
                    tri.v3 -= 1;
                }
            }
        }
    }
}

fn filter_mesh(vertices: &mut Vec<Vec3D>, tris: &mut Vec<Tri>) {
    println!("Removing redundant vertices...");
    //remove_redundant_vertices(vertices, tris);

    println!("Removing upside-down tris...");
    remove_upside_down_tris(vertices, tris);

    println!("Removing tris far above the camera...");
    remove_tris_far_above_camera(vertices, tris);

    println!("Removing unused vertices...");
    remove_unused_vertices(vertices, tris);
}

struct SmoothedParticle {
    pos: Vec2D,
    height: f32,
    area: f32
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
            height += particle.area * particle.height * (4.0 / (f32::consts::PI * INTERACTION_RADIUS_SQR.powf(4.0))) * (INTERACTION_RADIUS_SQR - d_sqr).powf(3.0);
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

fn mesh_to_grid(filename: &str, min_x: f32, max_x: f32, min_z: f32, max_z: f32, res_x: usize, res_z: usize) -> io::Result<Grid> {
    let (mut vertices, mut tris) = read_mesh_from_file(filename)?;
    filter_mesh(&mut vertices, &mut tris);

    let particles = to_particles(&vertices, &tris);

    println!("Generating terrain grid...");
    let mut grid = Grid::new(min_x, max_x, min_z, max_z, res_x, res_z);
    for grid_z in 0..res_z {
        for grid_x in 0..res_x {
            let x = lerp_f32(min_x, max_x, grid_x as f32 / res_x as f32);
            let z = lerp_f32(min_z, max_z, grid_z as f32 / res_x as f32);
            grid.set(grid_x, grid_z, terrain_height(&particles, Vec2D { x, y: -z }));
            //grid.set(grid_x, grid_z, terrain_gradient(&particles, Vec2D { x, y: -z }));
        }
    }

    Ok(grid)
}

struct Grid {
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    min_z: f32,
    max_z: f32,
    res_x: usize,
    res_z: usize,
    values: Vec<f32>
}

impl Grid {
    fn new(min_x: f32, max_x: f32, min_z: f32, max_z: f32, res_x: usize, res_z: usize) -> Grid {
        Grid {
            min_x,
            max_x,
            min_y: 0.0,
            max_y: 0.0,
            min_z,
            max_z,
            res_x,
            res_z,
            values: vec![0.0; res_x * res_z]
        }
    }

    fn set(&mut self, x: usize, z: usize, value: f32) {
        let old_min = self.min_y;
        let old_max = self.max_y;
        self.values[z * x + x] = value;
        self.min_y = value;
        self.max_y = value;
        if value > old_min {
            for value in &self.values {
                if *value < self.min_y {
                    self.min_y = *value;
                }
            }
        }
        if value < old_max {
            for value in &self.values {
                if *value > self.max_y {
                    self.max_y = *value;
                }
            }
        }
    }

    fn get(&self, x: usize, z: usize) -> f32 {
        self.values[z * x + x]
    }

    fn get_relative(&self, x: usize, z: usize) -> f32 {
        if (self.min_y - self.max_y).abs() < f32::EPSILON {
            0.0
        } else {
            (self.values[z * x + x] - self.min_y) / (self.max_y - self.min_y)
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{mesh_to_grid, lerp_f32};
    use plotters::drawing::IntoDrawingArea;
    use plotters::prelude::BitMapBackend;
    use plotters::style::{BLACK, HSLColor};

    #[test]
    fn test_mesh() {
        let min_x = -5.0;
        let max_x = 5.0;
        let min_z = -5.0;
        let max_z = 5.0;
        let res_x = 100;
        let res_z = 100;

        match mesh_to_grid("test_mesh.ply", min_x, max_x, min_z, max_z, res_x, res_z) {
            Ok(grid) => {
                let root = BitMapBackend::new("images/terrain.png", (res_x as u32, res_z as u32)).into_drawing_area();
                root.fill(&BLACK).unwrap();

                for grid_z in 0..res_z {
                    for grid_x in 0..res_x {
                        let lightness = grid.get_relative(grid_x, grid_z);
                        let c = HSLColor(lerp_f32(-120.0, 60.0, lightness) as f64, 1.0, lightness as f64);
                        root.draw_pixel((grid_x as i32, grid_z as i32), &c).unwrap();
                    }
                }
            },
            Err(_) => assert_eq!(true, false)
        }
    }
}
