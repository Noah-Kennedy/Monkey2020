#=
mesh_to_grid_option2:
- Julia version: 1.4.2
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-02-16
=#

include("mesh_to_grid_common.jl")

function triangle_uv(p::Vec2D, t1::Vec2D, t2::Vec2D, t3::Vec2D)
    b1 = t2 - t1
    b2 = t3 - t1
    q = p - t1
    v = wedge(b1, q) / -wedge(b2, b1)
    if v == 1.0
        return 0.0, 1.0
    end
    numer = q - b2 * v
    denom = b1 * (1 - v)
    u = abs(denom.x) > abs(denom.y) ? numer.x / denom.x : numer.y / denom.y
    u, v
end

function terrain_height(vertices::Vector{Vertex}, tris::Vector{Tri}, p::Vec2D)
    for tri in tris
        t1 = Vec2D(vertices[tri.v1].x, vertices[tri.v1].z)
        t2 = Vec2D(vertices[tri.v2].x, vertices[tri.v2].z)
        t3 = Vec2D(vertices[tri.v3].x, vertices[tri.v3].z)
        u, v = triangle_uv(p, t1, t2, t3)
        if u >= 0 && v >= 0 && u <= 1 && v <= 1
            return lerp(lerp(vertices[tri.v1].y, vertices[tri.v2].y, u), vertices[tri.v3].y, v)
        end
    end
    0.0
end

function terrain_gradient(vertices::Vector{Vertex}, tris::Vector{Tri}, p::Vec2D)
    d = 0.001
    grad_x = (terrain_height(vertices, tris, p + Vec2D(d, 0.0)) - terrain_height(vertices, tris, p - Vec2D(d, 0.0))) / 2d
    grad_y = (terrain_height(vertices, tris, p + Vec2D(0.0, d)) - terrain_height(vertices, tris, p - Vec2D(0.0, d))) / 2d
    sqrt(grad_x * grad_x + grad_y * grad_y)
end

function main()
    result = read_mesh_from_file("test_mesh.ply")
    if result == nothing
        return
    end
    vertices, tris = result

    filter_mesh!(vertices, tris)

    println("Generating plot...")
    grid_x = range(-5, 0, length = 100)
    grid_z = range(1, 6, length = 100)
    #grid_y = [terrain_height(vertices, tris, Vec2D(x, -z)) for z in grid_z, x in grid_x]
    grid_y = [terrain_gradient(vertices, tris, Vec2D(x, -z)) for z in grid_z, x in grid_x]

    x = Vector{Float32}(undef, length(vertices))
    y = Vector{Float32}(undef, length(vertices))
    z = Vector{Float32}(undef, length(vertices))
    for i in 1:length(vertices)
        x[i] = vertices[i].x
        y[i] = vertices[i].y
        z[i] = vertices[i].z
    end
    plot_3d = Plots.scatter(x, -z, y, leg = false, markersize = 0)
    Plots.plot!(-1:1, zeros(3), zeros(3), leg = false, seriescolor = :red)
    Plots.plot!(zeros(3), -1:1, zeros(3), leg = false, seriescolor = :green)
    Plots.plot!(zeros(3), zeros(3), -1:1, leg = false, seriescolor = :blue)
    plot_2d = Plots.scatter(x, -z, leg = false, markersize = 0)
    Plots.plot!(-1:1, zeros(3), leg = false, seriescolor = :red)
    Plots.plot!(zeros(3), -1:1, leg = false, seriescolor = :green)
    #plot_terrain = Plots.plot(grid_x, grid_z, grid_y, seriestype = :confourf)
    plot_terrain = Plots.heatmap(grid_y)
    plot = Plots.plot(plot_3d, plot_2d, plot_terrain, layout = Plots.@layout [[a; b] c{0.7w}])
    display(plot)
end

@time main()
