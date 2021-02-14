#=
mesh_to_grid_common:
- Julia version: 1.4.2
- Author: Wallace Watler <watlerathome@gmail.com>
- Date: 2021-01-24
=#

#=
import Pkg
Pkg.add(["Plots"])
=#

import Plots
import Base.+
import Base.-
import Base.*

struct Vec2D{T<:Real}
    x::T
    y::T
end

+(a::Vec2D, b::Vec2D) = Vec2D(a.x + b.x, a.y + b.y)
-(a::Vec2D, b::Vec2D) = Vec2D(a.x - b.x, a.y - b.y)
*(a::Real, b::Vec2D) = Vec2D(a * b.x, a * b.y)
*(b::Vec2D, a::Real) = *(a, b)
length_sqr(v::Vec2D) = v.x * v.x + v.y * v.y
wedge(a::Vec2D, b::Vec2D) = a.x * b.y - a.y * b.x

struct Vec3D{T<:Real}
    x::T
    y::T
    z::T
end

+(a::Vec3D, b::Vec3D) = Vec3D(a.x + b.x, a.y + b.y, a.z + b.z)
-(a::Vec3D, b::Vec3D) = Vec3D(a.x - b.x, a.y - b.y, a.z - b.z)
*(a::Real, b::Vec3D) = Vec3D(a * b.x, a * b.y, a * b.z)
*(b::Vec3D, a::Real) = *(a, b)
length_sqr(v::Vec3D) = v.x * v.x + v.y * v.y + v.z * v.z
cross(a::Vec3D, b::Vec3D) = Vec3D(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)

lerp(a::Real, b::Real, frac::Real) = a * (1 - frac) + b * frac
lerp(a::Vec2D, b::Vec2D, frac::Real) = Vec2D(a.x * (1 - frac) + b.x * frac, a.y * (1 - frac) + b.y * frac)
lerp(a::Vec3D, b::Vec3D, frac::Real) = Vec3D(a.x * (1 - frac) + b.x * frac, a.y * (1 - frac) + b.z * frac, a.z * (1 - frac) + b.z * frac)

const Vertex = Vec3D{Float32}

struct Tri
    v1::Int32
    v2::Int32
    v3::Int32
end

#=
Read mesh data from a binary PLY file.

Expected file format:
----Header----
"ply\n" - ASCII
"format binary_little_endian 1.0\n" - ASCII
"element vertex ####\n" - ASCII
"property float32 x\n" - ASCII
"property float32 y\n" - ASCII
"property float32 z\n" - ASCII
"element face ####\n" - ASCII
"property list uchar int vertex_indices\n" - ASCII
"end_header\n" - ASCII
----Data (variable size)----
Formatted according to header - binary
=#
function read_mesh_from_file(filename)
    println("Reading mesh from $filename...")

    vertices = Vector{Vertex}(undef, 0)
    tris = Vector{Tri}(undef, 0)

    open(filename, "r") do io
        # Read header
        if readline(io) != "ply"
            println("Error: not a PLY file")
            return
        end

        if readline(io) != "format binary_little_endian 1.0"
            println("Error: not formatted in binary little-endian 1.0")
            return
        end

        # Skip the comment
        readline(io)

        next_line = split(readline(io))
        if next_line[1] != "element" || next_line[2] != "vertex"
            println("Error: Unexpected header strings encountered")
            return
        end
        num_vertices = parse(Int32, next_line[3])

        if readline(io) != "property float32 x"
            println("Error: Unexpected header strings encountered")
            return
        end

        if readline(io) != "property float32 y"
            println("Error: Unexpected header strings encountered")
            return
        end

        if readline(io) != "property float32 z"
            println("Error: Unexpected header strings encountered")
            return
        end

        next_line = split(readline(io))
        if next_line[1] != "element" || next_line[2] != "face"
            println("Error: Unexpected header strings encountered")
            return
        end
        num_tris = parse(Int32, next_line[3])

        if readline(io) != "property list uchar int vertex_indices"
            println("Error: Unexpected header strings encountered")
            return
        end

        if readline(io) != "end_header"
            println("Error: end of header not found")
            return
        end

        # Read binary data
        for _ in 1:num_vertices
            x = ltoh(read(io, Float32))
            y = ltoh(read(io, Float32))
            z = ltoh(read(io, Float32))
            push!(vertices, Vertex(x, y, z))
        end
        for _ in 1:num_tris
            if ltoh(read(io, UInt8)) != 3
                println("Error: Found a non-tri polygon")
                return
            end
            # 1 is added since Julia uses 1-based indexing
            v1 = ltoh(read(io, Int32)) + 1
            v2 = ltoh(read(io, Int32)) + 1
            v3 = ltoh(read(io, Int32)) + 1
            push!(tris, Tri(v1, v2, v3))
        end
    end

    return vertices, tris
end

function remove_redundant_vertices!(vertices::Vector{Vertex}, tris::Vector{Tri})
    for i in reverse(2:length(vertices))
        for j in 1:(i - 1)
            if vertices[i] == vertices[j]
                deleteat!(vertices, i)
                for k in 1:length(tris)
                    if tris[k].v1 == i
                        tris[k] = Tri(j, tris[k].v2, tris[k].v3)
                    elseif tris[k].v2 == i
                        tris[k] = Tri(tris[k].v1, j, tris[k].v3)
                    elseif tris[k].v3 == i
                        tris[k] = Tri(tris[k].v1, tris[k].v2, j)
                    end
                    if tris[k].v1 > i
                        tris[k] = Tri(tris[k].v1 - 1, tris[k].v2, tris[k].v3)
                    end
                    if tris[k].v2 > i
                        tris[k] = Tri(tris[k].v1, tris[k].v2 - 1, tris[k].v3)
                    end
                    if tris[k].v3 > i
                        tris[k] = Tri(tris[k].v1, tris[k].v2, tris[k].v3 - 1)
                    end
                end
                break
            end
        end
    end
end

function remove_upside_down_tris!(vertices::Vector{Vertex}, tris::Vector{Tri})
    filter!(tri -> begin
        t1 = vertices[tri.v2] - vertices[tri.v1]
        t2 = vertices[tri.v3] - vertices[tri.v1]
        cross(t1, t2).y >= 0
    end, tris)

    for i in reverse(1:length(vertices))
        vertex_used = false
        for tri in tris
            if tri.v1 == i || tri.v2 == i || tri.v3 == i
                vertex_used = true
                break
            end
        end
        if !vertex_used
            deleteat!(vertices, i)
            for k in 1:length(tris)
                if tris[k].v1 > i
                    tris[k] = Tri(tris[k].v1 - 1, tris[k].v2, tris[k].v3)
                end
                if tris[k].v2 > i
                    tris[k] = Tri(tris[k].v1, tris[k].v2 - 1, tris[k].v3)
                end
                if tris[k].v3 > i
                    tris[k] = Tri(tris[k].v1, tris[k].v2, tris[k].v3 - 1)
                end
            end
        end
    end
end

function filter_mesh!(vertices::Vector{Vertex}, tris::Vector{Tri})
    println("Removing redundant vertices...")
    remove_redundant_vertices!(vertices, tris)

    println("Removing upside-down tris...")
    remove_upside_down_tris!(vertices, tris)
end
