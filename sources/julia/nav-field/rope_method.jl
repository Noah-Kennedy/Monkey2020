#=
nav_field:
- Julia version: 
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-01-17
=#

#=
import Pkg
Pkg.add(["Plots"])
=#

import Plots

unit_vec(dir) = [cos(dir), sin(dir)]

cubic_bezier(p0, p1, p2, p3, t) = p0 * (1 - t)^3 + p1 * 3t * (1 - t)^2 + p2 * 3t^2 * (1 - t) + p3 * t^3

smoothstep(frac, low, high) = low + (high - low) * (6frac^2 - 15frac + 10) * frac^3

function gradient(f, x, y)
    d = 0.001
    grad_x = (f(x + d, y) - f(x - d, y)) / 2d
    grad_y = (f(x, y + d) - f(x, y - d)) / 2d
    [grad_x, grad_y]
end

gradient(f, pos) = gradient(f, pos[1], pos[2])

function laplacian(f, x, y)
    d = 0.001
    laplacian_x = (gradient(f, x + d, y)[1] - gradient(f, x - d, y)[1]) / 2d
    laplacian_y = (gradient(f, x, y + d)[2] - gradient(f, x, y - d)[2]) / 2d
    laplacian_x + laplacian_y
end

laplacian(f, pos) = laplacian(f, pos[1], pos[2])

# Simulation parameters
const SPACE_SIZE = (5, 5)
const GRAD_GRID_0 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const GRAD_GRID_1 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const NUM_SEGMENTS = 100 # Num nodes = NUM_SEGMENTS + 1
const ITERS = 200
const dt = 1.0 / 20
const TENSION = 10.0
const GIVE = 20.0 # Any non-negative number is valid, higher = more give
const ALLOWED_GRAD = 0.36 # About a 20-degree incline
const PHEROMONE_VAP = 0.995
const ANT_ITERS = 2000

function clamp_within_space(x, y)
    while x < 0
        x += SPACE_SIZE[1]
    end
    while y < 0
        y += SPACE_SIZE[2]
    end
    x %= SPACE_SIZE[1]
    y %= SPACE_SIZE[2]
    (x, y)
end

clamp_within_space!(pos::Vector{Float64}) = (pos[1], pos[2]) = clamp_within_space(pos[1], pos[2])

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

terrain_elevation(pos) = terrain_elevation(pos[1], pos[2])

function terrain_gradient_mag(x, y)
    grad = gradient(terrain_elevation, x, y)
    sqrt(grad[1] * grad[1] + grad[2] * grad[2])
end

terrain_gradient_mag(pos) = terrain_gradient_mag(pos[1], pos[2])

const TERRAIN_PLOT_X = range(0, SPACE_SIZE[1], length = 100)
const TERRAIN_PLOT_Y = range(0, SPACE_SIZE[2], length = 100)
#const TERRAIN_PLOT_Z = [terrain_elevation(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
const TERRAIN_PLOT_Z = [terrain_gradient_mag(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
#const TERRAIN_PLOT_Z = [laplacian(terrain_elevation, x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
#const TERRAIN_PLOT_Z = [laplacian(terrain_gradient_mag, x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
function do_plot(points::Vector{Vector{Float64}}, fancy::Bool, current_pos::Vector{Float64}, target_pos::Vector{Float64})
    plot = Plots.plot(TERRAIN_PLOT_X, TERRAIN_PLOT_Y, TERRAIN_PLOT_Z, seriestype = (fancy ? :confourf : :contour), xticks=0:1:SPACE_SIZE[1], yticks=0:1:SPACE_SIZE[2])
    Plots.scatter!([p[1] for p in points], [p[2] for p in points], leg = false, seriescolor = :lime, markersize = 2)
    Plots.scatter!([current_pos[1], target_pos[1]], [current_pos[2], target_pos[2]], leg = false, seriescolor = :red, markersize = 3)
    plot
end

# Particle swarm to find minima, hopefully including the global minimum.
function find_minima(current_pos::Vector{Float64}, target_pos::Vector{Float64})
    particles = [smoothstep(rand(), current_pos, target_pos) + unit_vec(rand() * 2pi) * 2 * rand() for _ in 1:1000]
    vel = [zeros(2) for _ in 1:1000]
    accel = [zeros(2) for _ in 1:1000]
    for i in 1:length(particles)
        clamp_within_space!(particles[i])
        accel[i] = -gradient(terrain_gradient_mag, particles[i])
    end

    for _ in 1:ITERS
        display(do_plot(particles, false, current_pos, target_pos))
        for i in 1:length(particles)
            accel[i] = -gradient(terrain_gradient_mag, particles[i])
            vel[i] += accel[i] * dt
            vel[i] *= 0.2^dt # Linear drag; 20% of velocity remains after each second
            particles[i] += vel[i] * dt
            clamp_within_space!(particles[i])
        end
    end

    # Delete overlapping particles and those on too steep of slopes.
    for i in Iterators.reverse(1:length(particles))
        if terrain_gradient_mag(particles[i]) > ALLOWED_GRAD
            deleteat!(particles, i)
            continue
        end

        for j in 1:(i - 1)
            diff = particles[i] - particles[j]
            if diff[1] * diff[1] + diff[2] * diff[2] < 0.05^2
                deleteat!(particles, i)
                break
            end
        end
    end

    display(do_plot(particles, false, current_pos, target_pos))

    particles
end

function roulette(weights)
    d = rand() * sum(weights)
    for i in 1:length(weights)
        if d < weights[i]
            return i
        end
        d -= weights[i]
    end
    return length(weights)
end

function do_nav(current_pos::Vector{Float64}, current_dir::Float64, target_pos::Vector{Float64}, target_dir::Float64)
    waypoints = find_minima(current_pos, target_pos)
    for waypoint in waypoints
        clamp_within_space!(waypoint)
    end
    println("Num waypoints found: $(length(waypoints))")

    # Add the start and end points.
    insert!(waypoints, 1, current_pos - unit_vec(current_dir))
    insert!(waypoints, 2, deepcopy(current_pos))
    insert!(waypoints, 3, deepcopy(target_pos))
    insert!(waypoints, 4, target_pos + unit_vec(target_dir))

    # Distance and maximum gradient encountered between each pair of waypoints.
    safe_connections = zeros(Int64, length(waypoints))
    connection_length = zeros(length(waypoints), length(waypoints))
    max_grad = zeros(length(waypoints), length(waypoints))

    for at in 1:length(waypoints)
        max_grad[at, at] = terrain_gradient_mag(waypoints[at])
        for to in 1:(at - 1)
            connection_length[at, to] = connection_length[to, at] = hypot(waypoints[at][1] - waypoints[to][1], waypoints[at][2] - waypoints[to][2])
            max_grad[at, to] = max_grad[to, at] = maximum([terrain_gradient_mag(smoothstep(frac, waypoints[at], waypoints[to])) for frac in 0:min(1.0, 0.01 / connection_length[at, to]):1])
            if at != 1 && at != 4 && to != 1 && to != 4 && max_grad[at, to] <= ALLOWED_GRAD
                safe_connections[at] += 1
                safe_connections[to] += 1
            end
        end
    end

    # Delete waypoints that have less than two safe connections to other waypoints.
    keep = map(s -> s >= 2, safe_connections)

    # Indices 2 and 3 are the start and end points; they are allowed to have only one connection.
    # Indices 1 and 4 are only used for directional information, so they don't need any connections.
    keep[2] = safe_connections[2] >= 1
    keep[3] = safe_connections[3] >= 1
    keep[1] = true
    keep[4] = true

    if keep[2] == false || keep[3] == false
        println("No safe paths found")
        return
    end

    waypoints = waypoints[keep]
    safe_connections = safe_connections[keep]
    connection_length = connection_length[keep, keep]
    max_grad = max_grad[keep, keep]

    # Directional change between each triple of points.
    straightness = [begin
                        if from == at || at == to
                            0.0
                        else
                            w1 = waypoints[at] - waypoints[from]
                            w2 = waypoints[to] - waypoints[at]
                            w1 /= hypot(w1[1], w1[2])
                            w2 /= hypot(w2[1], w2[2])
                            (w1[1] * w2[1] + w1[2] * w2[2] + 1) / 2
                        end
                    end for to in 1:length(waypoints), at in 1:length(waypoints), from in 1:length(waypoints)]

    display(do_plot(waypoints, false, current_pos, target_pos))
    println("Num after deletion: $(length(waypoints))")

    # Begin ant colony optimization to find a path.
    pheromone = [((at == to || max_grad[at, to] > ALLOWED_GRAD) ? 0.0 : 1.0) for to in 1:length(waypoints), at in 1:length(waypoints)]

    best_path = []
    best_path_value = 0.0
    for _ in 1:ANT_ITERS
        path = [1, 2]
        path_length = 0.0
        min_straightness = Inf

        while path[end] != 3
            next_node = roulette(pheromone[path[end], :])
            path_length += connection_length[path[end], next_node]
            min_straightness = min(min_straightness, straightness[path[end - 1], path[end], next_node])
            push!(path, next_node)
        end
        min_straightness = min(min_straightness, straightness[path[end - 1], path[end], 4])
        push!(path, 4)

        path_value = min_straightness / path_length
        if path_value > best_path_value
            best_path = deepcopy(path[2:end-1])
            best_path_value = path_value
        end

        plot = do_plot(waypoints[2:end-1], false, current_pos, target_pos)
        Plots.plot!([waypoint[1] for waypoint in waypoints[path[2:end-1]]], [waypoint[2] for waypoint in waypoints[path[2:end-1]]], leg = false, seriescolor = :blue)
        Plots.plot!([waypoint[1] for waypoint in waypoints[best_path]], [waypoint[2] for waypoint in waypoints[best_path]], leg = false, seriescolor = :red)
        display(plot)

        for i in 3:(length(path) - 1)
            from = path[i - 2]
            at = path[i - 1]
            to = path[i]
            pheromone[at, to] += path_value
            pheromone[to, at] += path_value
        end
        pheromone .*= PHEROMONE_VAP
    end

    #=
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

    for _ in 1:ITERS
        do_plot(spline)
        terrain_grad_at_nodes = map(node -> terrain_gradient_mag(node[1], node[2]), spline)

        println("Max terrain grad: $(findmax(terrain_grad_at_nodes))")

        # Stop when all nodes' potentials are below some threshold.
        if all(grad -> grad <= ALLOWED_GRAD, terrain_grad_at_nodes)
            break
        end

        # Each node's acceleration depends on:
        #  - The cost function (moves downhill)
        #  - Tension between neighboring nodes
        #  - How much give it has
        accel = [zeros(2) for node in spline]
        for i in 2:(length(spline) - 1)
            d = 0.001
            grad_x = (terrain_gradient_mag(spline[i][1] + d, spline[i][2]) - terrain_gradient_mag(spline[i][1] - d, spline[i][2])) / 2d
            grad_y = (terrain_gradient_mag(spline[i][1], spline[i][2] + d) - terrain_gradient_mag(spline[i][1], spline[i][2] - d)) / 2d
            accel_terrain = -[grad_x, grad_y]

            g1 = spline[i - 1] - spline[i]
            g2 = spline[i + 1] - spline[i]
            accel_tension = (g1 + g2) * TENSION * NUM_SEGMENTS

            accel_give = (base_spline[i] - spline[i]) * TENSION * NUM_SEGMENTS

            accel[i] = accel_terrain * (spline_give[i]) + accel_tension + accel_give * (1 - spline_give[i])
            #accel[i] = accel_terrain + accel_tension
        end

        for i in 2:(length(spline) - 1)
            spline_vel[i] += accel[i] * dt
            spline_vel[i] *= 0.2^dt # Linear drag; 20% of velocity remains after each second
            spline[i] += spline_vel[i] * dt
        end
    end

    do_plot(spline, true)
    println()=#
end

@time do_nav([1.5, 0.5], 0.0pi / 180, [4.0, 4.0], 0.0pi / 180)
