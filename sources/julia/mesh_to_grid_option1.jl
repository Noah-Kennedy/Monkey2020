#=
mesh_to_grid_option1:
- Julia version: 
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-01-24
=#

include("mesh_to_grid_common.jl")

const INTERACTION_RADIUS_SQR = 0.10^2

struct SmoothedParticle
    pos::Vec2D{Float64}
    height::Float64
    area::Float64
end

function to_particles(vertices::Vector{Vertex}, tris::Vector{Tri})
    particles = Vector{SmoothedParticle}(undef, 0)

    for tri in tris
        t1 = Vec2D(vertices[tri.v1].x, vertices[tri.v1].z)
        t2 = Vec2D(vertices[tri.v2].x, vertices[tri.v2].z)
        t3 = Vec2D(vertices[tri.v3].x, vertices[tri.v3].z)

        # The centroid
        pos = lerp(lerp(t1, t2, 0.5), t3, 1 / 3)
        height = lerp(lerp(vertices[tri.v1].y, vertices[tri.v2].y, 0.5), vertices[tri.v3].y, 1 / 3)
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

function main()
    result = read_mesh_from_file("test_mesh.ply")
    if result == nothing
        return
    end
    vertices, tris = result

    filter_mesh!(vertices, tris)

    particles = to_particles(vertices, tris)

    println("Generating plot...")
    grid_x = range(-5, 0, length = 200)
    grid_z = range(1, 6, length = 200)
    #grid_y = [terrain_height(particles, Vec2D(x, -z)) for z in grid_z, x in grid_x]
    grid_y = [terrain_gradient(particles, Vec2D(x, -z)) for z in grid_z, x in grid_x]

    x = Vector{Float32}(undef, length(vertices))
    y = Vector{Float32}(undef, length(vertices))
    z = Vector{Float32}(undef, length(vertices))
    for i in 1:length(vertices)
        x[i] = vertices[i].x
        y[i] = vertices[i].y
        z[i] = vertices[i].z
    end
    plot_3d = Plots.scatter(x, -z, y, leg = false, markersize = 1)
    Plots.plot!(-1:1, zeros(3), zeros(3), leg = false, seriescolor = :red)
    Plots.plot!(zeros(3), -1:1, zeros(3), leg = false, seriescolor = :green)
    Plots.plot!(zeros(3), zeros(3), -1:1, leg = false, seriescolor = :blue)
    plot_2d = Plots.scatter([vertex.x for vertex in filter(v -> v.y <= 1, vertices)], [-vertex.z for vertex in filter(v -> v.y <= 1, vertices)], leg = false, markersize = 0)
    Plots.plot!(-1:1, zeros(3), leg = false, seriescolor = :red)
    Plots.plot!(zeros(3), -1:1, leg = false, seriescolor = :green)
    plot_terrain = Plots.plot(grid_x, grid_z, grid_y, seriestype = :confourf)
    plot = Plots.plot(plot_3d, plot_2d, plot_terrain, layout = Plots.@layout [[a; b] c{0.7w}])
    display(plot)
end

@time main()
