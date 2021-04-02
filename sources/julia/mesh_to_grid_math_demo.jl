#=
mesh_to_grid_math_demo:
- Julia version: 1.4.2
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-02-16
=#

include("mesh_to_grid_common.jl")

const INTERACTION_RADIUS_SQR = 1.0^2

struct SmoothedParticle
    pos::Vec2D{Float64}
    height::Float64
    area::Float64
end

function to_particles(vertices::Vector{Vertex}, tris::Vector{Tri})
    particles = Vector{SmoothedParticle}(undef, 0)

    for tri in tris
        t1 = Vec2D(vertices[tri.v1].x, vertices[tri.v1].y)
        t2 = Vec2D(vertices[tri.v2].x, vertices[tri.v2].y)
        t3 = Vec2D(vertices[tri.v3].x, vertices[tri.v3].y)

        # The centroid
        pos = (t1 + t2 + t3) * (1 / 3)
        height = (vertices[tri.v1].y + vertices[tri.v2].y + vertices[tri.v3].y) * (1 / 3)
        area = abs(wedge(t2 - t1, t3 - t1)) / 2

        if height <= 1
            push!(particles, SmoothedParticle(pos, height, area))
        end
    end

    particles
end

function terrain_height(particles::Vector{SmoothedParticle}, p::Vec2D)
    height = 0.0
    for particle in particles
        d_sqr = length_sqr(p - particle.pos)
        if d_sqr < INTERACTION_RADIUS_SQR
            height += particle.area * particle.height * (4 / (pi * INTERACTION_RADIUS_SQR^4)) * (INTERACTION_RADIUS_SQR - d_sqr)^3
        end
    end
    height
end

function terrain_gradient(particles::Vector{SmoothedParticle}, p::Vec2D)
    d = 0.001
    grad_x = (terrain_height(particles, p + Vec2D(d, 0.0)) - terrain_height(particles, p - Vec2D(d, 0.0))) / 2d
    grad_y = (terrain_height(particles, p + Vec2D(0.0, d)) - terrain_height(particles, p - Vec2D(0.0, d))) / 2d
    sqrt(grad_x * grad_x + grad_y * grad_y)
end

function centroid(vertices::Vector{Vertex}, tri::Tri)
    t1 = Vec3D(vertices[tri.v1].x, vertices[tri.v1].y, vertices[tri.v1].z)
    t2 = Vec3D(vertices[tri.v2].x, vertices[tri.v2].y, vertices[tri.v2].z)
    t3 = Vec3D(vertices[tri.v3].x, vertices[tri.v3].y, vertices[tri.v3].z)
    (t1 + t2 + t3) * (1 / 3)
end

function main()
    vertices = [Vertex(6.0, 0.0, 2.0), Vertex(4.0, 2.0, 4.0), Vertex(3.0, -1.0, 3.0)]
    tris = [Tri(1, 2, 3)]
    particles = to_particles(vertices, tris)

    x = Vector{Float32}(undef, length(vertices) + 1)
    y = Vector{Float32}(undef, length(vertices) + 1)
    z = Vector{Float32}(undef, length(vertices) + 1)
    for i in 1:length(vertices)
        x[i] = vertices[i].x
        y[i] = vertices[i].y
        z[i] = vertices[i].z
    end
    x[end] = vertices[1].x
    y[end] = vertices[1].y
    z[end] = vertices[1].z

    println("Generating plot...")

    # Plot smoothed particle
    grid_x = range(2.5, 6.5, length = 50)
    grid_y = range(-1.5, 2.5, length = 50)
    plot_3d = Plots.plot(grid_x, grid_y, (x, y) -> terrain_height(particles, Vec2D(x, y)), st = :wireframe)

    # Plot triangle + centroid
    Plots.plot!(x, y, z, leg = false, markersize = 3, markershape = :circle)
    c = centroid(vertices, tris[1])
    Plots.scatter!([c.x], [c.y], [c.z], leg = false, seriescolor = :blue)

    # Plot projected triangle + centroid
    proj_vers = map(vertex -> Vec2D(vertex.x, vertex.y), vertices)
    Plots.plot!(push!([proj_ver.x for proj_ver in proj_vers], proj_vers[1].x), push!([proj_ver.y for proj_ver in proj_vers], proj_vers[1].y), zeros(Float32, length(proj_vers) + 1), leg = false, markersize = 3, markershape = :circle, seriescolor = :lime)
    Plots.scatter!([c.x], [c.y], [0.0], leg = false)
    display(plot_3d)

    # Plot axes
    Plots.plot!(-1:1, zeros(3), zeros(3), leg = false, seriescolor = :red)
    Plots.plot!(zeros(3), -1:1, zeros(3), leg = false, seriescolor = :green)
    Plots.plot!(zeros(3), zeros(3), -1:1, leg = false, seriescolor = :blue)
end

main()
