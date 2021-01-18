#=
nav_field:
- Julia version: 
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-01-16
=#

#=
import Pkg
Pkg.add(["Plots"])
=#

import Plots

unit_vec(dir) = [cos(dir), sin(dir)]

cubic_bezier(p0, p1, p2, p3, t) = p0 * (1 - t)^3 + p1 * 3t * (1 - t)^2 + p2 * 3t^2 * (1 - t) + p3 * t^3

smoothstep(frac, low, high) = low + (6frac^2 - 15frac + 10) * frac^3 * (high - low)

# Simulation parameters
const SPACE_SIZE = (5, 5)
const GRAD_GRID_0 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const GRAD_GRID_1 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const NUM_SEGMENTS = 100 # Num nodes = NUM_SEGMENTS + 1
const ITERS = 1000
const dt = 1.0 / 60
const TENSION = 10.0
const GIVE = 20.0 # Any non-negative number is valid, higher = more give
const CONDUCTIVITY = 0.97 # How quickly heat transfers between nodes; range [0.0 - 1.0]
const HEAT_ACCEL = 200.0 # Heat acceleration multiplier
const ALLOWED_GRAD = 0.36 # About a 20-degree incline

# Preallocated arrays
const pn_point = zeros(2)
const corner_0 = zeros(2)
const corner_1 = zeros(2)
const corner_2 = zeros(2)
const corner_3 = zeros(2)
function perlin_noise(x, y, grad_grid)
    while x < 0
        x += SPACE_SIZE[1]
    end
    while y < 0
        y += SPACE_SIZE[2]
    end
    x = x % SPACE_SIZE[1]
    y = y % SPACE_SIZE[2]

    pn_point .= [x, y]
    grid_x = floor(Int, x)
    grid_y = floor(Int, y)
    (corner_0[1], corner_0[2]) = (grid_x, grid_y)
    (corner_1[1], corner_1[2]) = (grid_x + 1, grid_y)
    (corner_2[1], corner_2[2]) = (grid_x, grid_y + 1)
    (corner_3[1], corner_3[2]) = (grid_x + 1, grid_y + 1)
    grad_0 = grad_grid[grid_x + 1, grid_y + 1]
    grad_1 = grad_grid[(grid_x + 1) % SPACE_SIZE[1] + 1, grid_y + 1]
    grad_2 = grad_grid[grid_x + 1, (grid_y + 1) % SPACE_SIZE[2] + 1]
    grad_3 = grad_grid[(grid_x + 1) % SPACE_SIZE[1] + 1, (grid_y + 1) % SPACE_SIZE[2] + 1]
    dot_0 = (pn_point[1] - corner_0[1]) * grad_0[1] + (pn_point[2] - corner_0[2]) * grad_0[2]
    dot_1 = (pn_point[1] - corner_1[1]) * grad_1[1] + (pn_point[2] - corner_1[2]) * grad_1[2]
    dot_2 = (pn_point[1] - corner_2[1]) * grad_2[1] + (pn_point[2] - corner_2[2]) * grad_2[2]
    dot_3 = (pn_point[1] - corner_3[1]) * grad_3[1] + (pn_point[2] - corner_3[2]) * grad_3[2]
    frac_x = x - grid_x
    frac_y = y - grid_y
    smoothstep(frac_y, smoothstep(frac_x, dot_0, dot_1), smoothstep(frac_x, dot_2, dot_3))
end

function terrain_elevation(x, y)
    # Natural terrain
    fractal_noise = perlin_noise(x, y, GRAD_GRID_0) + perlin_noise(2x, 2y, GRAD_GRID_1) / 2
    10^fractal_noise / 10

    # Plateau
    #1 / ((x - 2.5)^10 + 1) * 1 / ((y - 2.5)^10 + 1)
end

function terrain_gradient(x, y)
    d = 0.001
    grad_x = (terrain_elevation(x + d, y) - terrain_elevation(x - d, y)) / 2d
    grad_y = (terrain_elevation(x, y + d) - terrain_elevation(x, y - d)) / 2d
    [grad_x, grad_y]
end

function terrain_gradient_mag(x, y)
    grad = terrain_gradient(x, y)
    sqrt(grad[1] * grad[1] + grad[2] * grad[2])
end

function terrain_laplacian(x, y)
    d = 0.001
    laplacian_x = (terrain_gradient(x + d, y)[1] - terrain_gradient(x - d, y)[1]) / 2d
    laplacian_y = (terrain_gradient(x, y + d)[2] - terrain_gradient(x, y - d)[2]) / 2d
    laplacian_x + laplacian_y
end

function terrain_gradient_mag_gradient(x, y)
    d = 0.001
    grad_x = (terrain_gradient_mag(x + d, y) - terrain_gradient_mag(x - d, y)) / 2d
    grad_y = (terrain_gradient_mag(x, y + d) - terrain_gradient_mag(x, y - d)) / 2d
    [grad_x, grad_y]
end

function terrain_gradient_mag_laplacian(x, y)
    d = 0.001
    laplacian_x = (terrain_gradient_mag_gradient(x + d, y)[1] - terrain_gradient_mag_gradient(x - d, y)[1]) / 2d
    laplacian_y = (terrain_gradient_mag_gradient(x, y + d)[2] - terrain_gradient_mag_gradient(x, y - d)[2]) / 2d
    laplacian_x + laplacian_y
end

function cost(x, y)
    terrain_gradient_mag(x, y)
end

const TERRAIN_PLOT_X = range(0, SPACE_SIZE[1], length = 100)
const TERRAIN_PLOT_Y = range(0, SPACE_SIZE[2], length = 100)
#const TERRAIN_PLOT_Z = [terrain_elevation(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
#const TERRAIN_PLOT_Z = [terrain_gradient_mag(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
#const TERRAIN_PLOT_Z = [terrain_laplacian(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
#const TERRAIN_PLOT_Z = [terrain_gradient_mag_laplacian(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
const TERRAIN_PLOT_Z = [cost(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
function do_plot(spline::Vector{Vector{Float64}})
    plot = Plots.plot(TERRAIN_PLOT_X, TERRAIN_PLOT_Y, TERRAIN_PLOT_Z, seriestype = :contour, xticks=0:1:SPACE_SIZE[1], yticks=0:1:SPACE_SIZE[2])
    if NUM_SEGMENTS <= 100
        Plots.plot!([node[1] for node in spline], [node[2] for node in spline], leg = false, seriescolor = :lime, markershapes = :circle, markersize = 2)
    else
        Plots.plot!([node[1] for node in spline], [node[2] for node in spline], leg = false, seriescolor = :lime)
    end
    display(plot)
end

function do_fancy_plot(spline::Vector{Vector{Float64}})
    plot = Plots.plot(TERRAIN_PLOT_X, TERRAIN_PLOT_Y, TERRAIN_PLOT_Z, seriestype = :contourf, xticks=0:1:SPACE_SIZE[1], yticks=0:1:SPACE_SIZE[2])
    if NUM_SEGMENTS <= 100
        Plots.plot!([node[1] for node in spline], [node[2] for node in spline], leg = false, seriescolor = :lime, markershapes = :circle, markersize = 2)
    else
        Plots.plot!([node[1] for node in spline], [node[2] for node in spline], leg = false, seriescolor = :lime)
    end
    display(plot)
end

function do_nav(current_pos::Vector{Float64}, current_dir::Float64, target_pos::Vector{Float64}, target_dir::Float64)
    p1 = current_pos + unit_vec(current_dir)
    p2 = target_pos - unit_vec(target_dir)

    # Create initial spline, divided into nodes.
    # This is the ideal path if nothing were in the way.
    base_spline = map(t -> cubic_bezier(current_pos, p1, p2, target_pos, t), collect(0:(1 / NUM_SEGMENTS):1))

    # How much each node in the altered path should stick to the ideal spline.
    # This is necessary to ensure that the endpoints' orientations do not change much.
    # Nodes in the middle have the most freedom; those towards the ends have less freedom.
    spline_give = [1 - abs(2t - 1)^GIVE for t in collect(0:(1 / NUM_SEGMENTS):1)]

    # Altered spline to be calculated; accounts for obstacles in the terrain.
    spline = deepcopy(base_spline)
    spline_vel = [zeros(2) for node in spline] # Velocity of each node
    spline_heat = zeros(length(spline)) # "Heat" in each node

    for _ in 1:ITERS
        do_plot(spline)
        terrain_grad_at_nodes = map(node -> terrain_gradient_mag(node[1], node[2]), spline)

        # The magnitude of the terrain gradient acts as a heat flux.
        # The ends of the path are heat sinks, so they never heat up.
        spline_heat[2:end-1] += terrain_grad_at_nodes[2:end-1] * dt

        println("Max terrain grad: $(findmax(terrain_grad_at_nodes))")

        # Stop when all nodes' potentials are below some threshold.
        if all(grad -> grad <= ALLOWED_GRAD, terrain_grad_at_nodes)
            break
        end

        # Each node's acceleration depends on:
        #  - The cost function (moves downhill)
        #  - Tension between neighboring nodes
        #  - How much give it has
        #  - Its heat
        accel = [zeros(2) for node in spline]
        neighbor_avg_heat = zeros(length(spline))
        for i in 2:(length(spline) - 1)
            d = 0.001
            grad_x = (cost(spline[i][1] + d, spline[i][2]) - cost(spline[i][1] - d, spline[i][2])) / 2d
            grad_y = (cost(spline[i][1], spline[i][2] + d) - cost(spline[i][1], spline[i][2] - d)) / 2d
            accel_terrain = -[grad_x, grad_y]

            g1 = spline[i - 1] - spline[i]
            g2 = spline[i + 1] - spline[i]
            accel_tension = (g1 + g2) * TENSION * NUM_SEGMENTS

            accel_give = (base_spline[i] - spline[i]) * TENSION * NUM_SEGMENTS

            g1_mag = hypot(g1[1], g1[2])
            g2_mag = hypot(g2[1], g2[2])
            g1_unit = g1 / g1_mag
            g2_unit = g2 / g2_mag
            k_cross_g2 = [-g2_unit[2], g2_unit[1]]
            k_cross_g1 = [-g1_unit[2], g1_unit[1]]
            accel_heat = (k_cross_g2 * (spline_heat[i - 1] - spline_heat[i]) - k_cross_g1 * (spline_heat[i + 1] - spline_heat[i])) * spline_heat[i] * (g1_unit[1] * g2_unit[2] - g1_unit[2] * g2_unit[1]) * HEAT_ACCEL

            accel[i] = accel_terrain * (spline_give[i]) + accel_tension + accel_give * (1 - spline_give[i])
            #accel[i] = accel_terrain + accel_tension
            #accel[i] = accel_terrain + accel_tension + accel_heat

            neighbor_avg_heat[i] += (spline_heat[i - 1] + spline_heat[i] + spline_heat[i + 1]) / 3
        end

        for i in 2:(length(spline) - 1)
            # Nodes tend towards heat equilibrium.
            spline_heat[i] += (neighbor_avg_heat[i] - spline_heat[i]) * CONDUCTIVITY^(1 / dt)

            spline_vel[i] += accel[i] * dt
            spline_vel[i] *= 0.2^dt # Linear drag; 20% of velocity remains after each second
            spline[i] += spline_vel[i] * dt
        end
    end

    do_fancy_plot(spline)
    display(spline_heat)
    println()
end

function main()
    do_nav([1.5, 0.5], 0.0pi / (180), [4.0, 4.0], 0.0pi / (180))
end

@time main()
