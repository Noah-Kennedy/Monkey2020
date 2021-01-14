#=
nav_field:
- Julia version: 
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-01-14
=#

#=
import Pkg
Pkg.add(["Plots"])
=#

import Plots

unit_vec(dir) = [cos(dir), sin(dir)]

cubic_bezier(p0, p1, p2, p3, t) = p0 * (1 - t)^3 + p1 * 3t * (1 - t)^2 + p2 * 3t^2 * (1 - t) + p3 * t^3

smoothstep(frac) = 6frac^5 - 15frac^4 + 10frac^3

# Simulation parameters
const SPACE_SIZE = (5, 5)
const NUM_SEGMENTS = 100 # Num nodes = NUM_SEGMENTS + 1
const ITERS = 1
const dt = 1.0 / 1
const GRAD_GRID_0 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const GRAD_GRID_1 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]

function perlin_noise(x, y, grad_grid)
    while x < 0
        x += SPACE_SIZE[1]
    end
    while y < 0
        y += SPACE_SIZE[2]
    end
    x = x % SPACE_SIZE[1]
    y = y % SPACE_SIZE[2]

    point = [x, y]
    grid_x = floor(Int, x)
    grid_y = floor(Int, y)
    corner_0 = [grid_x, grid_y]
    corner_1 = [grid_x + 1, grid_y]
    corner_2 = [grid_x, grid_y + 1]
    corner_3 = [grid_x + 1, grid_y + 1]
    grad_0 = grad_grid[grid_x + 1, grid_y + 1]
    grad_1 = grad_grid[(grid_x + 1) % SPACE_SIZE[1] + 1, grid_y + 1]
    grad_2 = grad_grid[grid_x + 1, (grid_y + 1) % SPACE_SIZE[2] + 1]
    grad_3 = grad_grid[(grid_x + 1) % SPACE_SIZE[1] + 1, (grid_y + 1) % SPACE_SIZE[2] + 1]
    dot_0 = sum((point - corner_0) .* grad_0)
    dot_1 = sum((point - corner_1) .* grad_1)
    dot_2 = sum((point - corner_2) .* grad_2)
    dot_3 = sum((point - corner_3) .* grad_3)
    frac_x = x - grid_x
    frac_y = y - grid_y
    lerp_x0 = dot_0 + smoothstep(frac_x) * (dot_1 - dot_0)
    lerp_x1 = dot_2 + smoothstep(frac_x) * (dot_3 - dot_2)
    lerp_x0 + smoothstep(frac_y) * (lerp_x1 - lerp_x0)
end

function terrain_elevation(x, y)
    fractal_noise = perlin_noise(x, y, GRAD_GRID_0) + perlin_noise(2x, 2y, GRAD_GRID_1) / 2
    10^fractal_noise / 10
end

function terrain_gradient_mag(x, y)
    d = 0.001
    grad_x = (terrain_elevation(x + d, y) - terrain_elevation(x - d, y)) / 2d
    grad_y = (terrain_elevation(x, y + d) - terrain_elevation(x, y - d)) / 2d
    sqrt(grad_x * grad_x + grad_y * grad_y)
end

function do_plot(spline::Vector{Vector{Float64}})
    #plot = Plots.plot(range(0, SPACE_SIZE[1], length = 100), range(0, SPACE_SIZE[1], length = 100), terrain_elevation, seriestype = :contourf, xticks=0:1:SPACE_SIZE[1], yticks=0:1:SPACE_SIZE[2])
    plot = Plots.plot(range(0, SPACE_SIZE[1], length = 100), range(0, SPACE_SIZE[1], length = 100), terrain_gradient_mag, seriestype = :contourf, xticks=0:1:SPACE_SIZE[1], yticks=0:1:SPACE_SIZE[2])
    Plots.scatter!([node[1] for node in spline], [node[2] for node in spline], leg = false, seriescolor = :lime)
    display(plot)
    sleep(dt)
end

function do_nav(current_pos::Vector{Float64}, current_dir::Float64, target_pos::Vector{Float64}, target_dir::Float64)
    p1 = current_pos + unit_vec(current_dir)
    p2 = target_pos - unit_vec(target_dir)

    # Create initial spline, divided into nodes
    spline = map(t -> cubic_bezier(current_pos, p1, p2, target_pos, t), collect(0:(1 / NUM_SEGMENTS):1))

    # TODO: Stop when all nodes' potentials are below some threshold
    for _ in 1:ITERS
        do_plot(spline)
        terrain_grad_at_nodes = map(node -> terrain_gradient_mag(node[1], node[2]), spline)
        # TODO: Compare adjacent nodes' gradient values to determine which direction each should move in
        # Probably want to add some kind of elastic force to keep nodes near each other as well
    end

    do_plot(spline)
end

function main()
    do_nav([1.2, 0.5], 0.0pi / (180), [4.0, 4.0], 0.0pi / (180))
end

main()
