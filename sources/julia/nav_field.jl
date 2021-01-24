#=
nav_field:
- Julia version: 
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-01-19
=#

#=
import Pkg
Pkg.add(["Plots"])
=#

import Plots

function unit_vec!(a, dir)
    a[1] = cos(dir)
    a[2] = sin(dir)
    a
end

unit_vec(dir) = unit_vec!(zeros(2), dir)

cubic_bezier(p0, p1, p2, p3, t) = p0 * (1 - t)^3 + p1 * 3t * (1 - t)^2 + p2 * 3t^2 * (1 - t) + p3 * t^3

smoothstep(frac::Number, low::Number, high::Number) = low + (high - low) * (6frac^2 - 15frac + 10) * frac^3

function smoothstep!(a, frac, low, high)
    a[1] = smoothstep(frac, low[1], high[1])
    a[2] = smoothstep(frac, low[2], high[2])
    a
end

function gradient!(a, f, x, y)
    d = 0.001
    a[1] = (f(x + d, y) - f(x - d, y)) / 2d
    a[2] = (f(x, y + d) - f(x, y - d)) / 2d
    a
end

gradient!(a, f, pos) = gradient!(a, f, pos[1], pos[2])

gradient(f, x, y) = gradient!(zeros(2), f, x, y)

gradient(f, pos) = gradient!(zeros(2), f, pos)

# Simulation parameters
const SPACE_SIZE = (5, 5)
const GRAD_GRID_0 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const GRAD_GRID_1 = [unit_vec(2pi * rand()) for x in 1:SPACE_SIZE[1], y in 1:SPACE_SIZE[2]]
const ITERS = 200
const dt = 1.0 / 20
const ALLOWED_GRAD = 0.36 # About a 20-degree incline
const PHEROMONE_VAP = 0.995
const ANT_ITERS = 1000
const NUM_ANTS = 100
const STRAIGHTNESS_POW = 1.0 # (0.0, Inf]; make it higher to put more importance on finding paths without sharp turns.

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

const tgm_grad = zeros(2)
function terrain_gradient_mag(x, y)
    gradient!(tgm_grad, terrain_elevation, x, y)
    sqrt(tgm_grad[1] * tgm_grad[1] + tgm_grad[2] * tgm_grad[2])
end

terrain_gradient_mag(pos) = terrain_gradient_mag(pos[1], pos[2])

const TERRAIN_PLOT_X = range(0, SPACE_SIZE[1], length = 100)
const TERRAIN_PLOT_Y = range(0, SPACE_SIZE[2], length = 100)
#const TERRAIN_PLOT_Z = [terrain_elevation(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
const TERRAIN_PLOT_Z = [terrain_gradient_mag(x, y) for y in TERRAIN_PLOT_Y, x in TERRAIN_PLOT_X]
function do_plot(points::Vector{Vector{Float64}}, fancy::Bool, current_pos::Vector{Float64}, target_pos::Vector{Float64})
    plot = Plots.plot(TERRAIN_PLOT_X, TERRAIN_PLOT_Y, TERRAIN_PLOT_Z, seriestype = (fancy ? :confourf : :contour), xticks=0:1:SPACE_SIZE[1], yticks=0:1:SPACE_SIZE[2])
    Plots.scatter!([p[1] for p in points], [p[2] for p in points], leg = false, seriescolor = :lime, markersize = 2)
    Plots.scatter!([current_pos[1], target_pos[1]], [current_pos[2], target_pos[2]], leg = false, seriescolor = :red, markersize = 3)
    plot
end

const NUM_PARTICLES = 200
# Particle swarm to find minima, hopefully including the global minimum.
function find_minima(current_pos::Vector{Float64}, target_pos::Vector{Float64})
    particles = [begin
                     frac = rand()
                     dir = rand() * 2pi
                     dist = rand() * 2
                     x = smoothstep(frac, current_pos[1], target_pos[1]) + cos(dir) * dist
                     y = smoothstep(frac, current_pos[2], target_pos[2]) + sin(dir) * dist
                     [x, y]
                 end for _ in 1:NUM_PARTICLES]
    vel = [zeros(2) for _ in 1:NUM_PARTICLES]
    accel = [zeros(2) for _ in 1:NUM_PARTICLES]
    for i in 1:NUM_PARTICLES
        clamp_within_space!(particles[i])
        gradient!(accel[i], terrain_gradient_mag, particles[i])
        accel[i] .*= -1
    end

    for _ in 1:ITERS
        #display(do_plot(particles, false, current_pos, target_pos))
        for i in 1:NUM_PARTICLES
            gradient!(accel[i], terrain_gradient_mag, particles[i])
            accel[i] .*= -1
            vel[i] .+= accel[i] .* dt
            vel[i] .*= 0.2^dt # Linear drag; 20% of velocity remains after each second
            particles[i] .+= vel[i] .* dt
            clamp_within_space!(particles[i])
        end
    end

    diff = zeros(2)
    # Delete overlapping particles and those on too steep of slopes.
    for i in Iterators.reverse(1:NUM_PARTICLES)
        if terrain_gradient_mag(particles[i]) > ALLOWED_GRAD
            deleteat!(particles, i)
            continue
        end

        for j in 1:(i - 1)
            diff .= particles[i] .- particles[j]
            if diff[1] * diff[1] + diff[2] * diff[2] < 0.05^2
                deleteat!(particles, i)
                break
            end
        end
    end

    #display(do_plot(particles, false, current_pos, target_pos))

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

mutable struct Ant
    from::Int64
    at::Int64
    to::Int64
    to_target::Bool
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

    temp = zeros(2)
    for at in 1:length(waypoints)
        max_grad[at, at] = terrain_gradient_mag(waypoints[at])
        for to in 1:(at - 1)
            connection_length[at, to] = connection_length[to, at] = hypot(waypoints[at][1] - waypoints[to][1], waypoints[at][2] - waypoints[to][2])
            max_grad[at, to] = max_grad[to, at] = maximum([terrain_gradient_mag(smoothstep!(temp, frac, waypoints[at], waypoints[to])) for frac in 0:min(1.0, 0.01 / connection_length[at, to]):1])
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

    # Preallocations
    w1 = zeros(2)
    w2 = zeros(2)
    weights = zeros(length(waypoints))
    path = [1, 2]

    # Directional change between each triple of points.
    straightness = [begin
                        if from == at || at == to
                            0.0
                        else
                            w1 .= waypoints[at] .- waypoints[from]
                            w2 .= waypoints[to] .- waypoints[at]
                            w1 ./= hypot(w1[1], w1[2])
                            w2 ./= hypot(w2[1], w2[2])
                            clamp((w1[1] * w2[1] + w1[2] * w2[2] + 1) / 2, 0.0, 1.0)
                        end
                    end for to in 1:length(waypoints), at in 1:length(waypoints), from in 1:length(waypoints)]

    display(do_plot(waypoints, false, current_pos, target_pos))
    println("Num after deletion: $(length(waypoints))")

    # Begin ant colony optimization to find a path.
    # Pheromone laid by ants travelling from the current position.
    pheromone_start = [((at == to || max_grad[at, to] > ALLOWED_GRAD) ? 0.0 : 1.0) for to in 1:length(waypoints), at in 1:length(waypoints)]

    # Pheromone laid by ants travelling from the target position.
    pheromone_end = copy(pheromone_start)

    best_path = []
    best_path_value = 0.0
    ants = [Ant(1, 2, 3, true) for _ in 1:NUM_ANTS]
    for _ in 1:ANT_ITERS
        # Simulate an ant following a path from start to finish.
        empty!(path)
        push!(path, 1)
        push!(path, 2)
        path_length = 0.0
        min_straightness = Inf

        while path[end] != 3
            to = roulette(pheromone_end[path[end], :] .* straightness[path[end - 1], path[end], :].^STRAIGHTNESS_POW)
            path_length += connection_length[path[end], to]
            min_straightness = min(min_straightness, straightness[path[end - 1], path[end], to])
            push!(path, to)
        end
        min_straightness = min(min_straightness, straightness[path[end - 1], path[end], 4])
        push!(path, 4)

        path_value = min_straightness^STRAIGHTNESS_POW / path_length
        if path_value > best_path_value
            best_path = copy(path[2:end-1])
            best_path_value = path_value
        end

        #plot = do_plot(waypoints[2:end-1], false, current_pos, target_pos)
        #Plots.plot!([waypoint[1] for waypoint in waypoints[path[2:end-1]]], [waypoint[2] for waypoint in waypoints[path[2:end-1]]], leg = false, seriescolor = :blue)
        #Plots.plot!([waypoint[1] for waypoint in waypoints[best_path]], [waypoint[2] for waypoint in waypoints[best_path]], leg = false, seriescolor = :red)
        #display(plot)

        # Calculate next connections to traverse.
        for ant in ants
            weights .= (ant.to_target ? pheromone_end : pheromone_start)[ant.at, :] .* straightness[ant.from, ant.at, :].^STRAIGHTNESS_POW
            ant.to = roulette(weights)
        end

        # Update ants and leave pheromone.
        for ant in ants
            ant.from = ant.at
            ant.at = ant.to
            (ant.to_target ? pheromone_start : pheromone_end)[ant.from, ant.at] += 1 / connection_length[ant.from, ant.at]

            # Switch direction if necessary.
            if ant.to_target && ant.at == 3
                ant.to_target = false
                ant.from = 4
            elseif !ant.to_target && ant.at == 2
                ant.to_target = true
                ant.from = 1
            end
        end

        pheromone_start .*= PHEROMONE_VAP
        pheromone_end .*= PHEROMONE_VAP
    end

    plot = do_plot(waypoints[2:end-1], false, current_pos, target_pos)
    Plots.plot!([waypoint[1] for waypoint in waypoints[best_path]], [waypoint[2] for waypoint in waypoints[best_path]], leg = false, seriescolor = :red)
    display(plot)
end

@time do_nav([1.5, 0.5], 0.0pi / 180, [4.0, 4.0], 0.0pi / 180)
