--!native
--!optimize 2

-- Benchmarks the hot paths: `lune run bench`
-- Every number is nanoseconds per operation (per point for the batch samplers), best of several runs.
-- Lune runs the same Luau VM and native codegen as Roblox, but absolute numbers will differ in Studio.
-- The last column is the Path baseline (./Path.luau): straight segments through the same points, so compare it
-- with Linear.

local Path = require("./Path")
local Splynx = require("../src")

local POINT_COUNT = 61 -- 3k + 1, so the Bezier spline uses every point
local BATCH_SIZE = 1000
local SAMPLED_SPACING = 0.5
-- Rounded's reach and Filleted's radius
local CORNER_SIZE = 20
-- Units moving along the spline at once, for the batch lookups
local UNIT_COUNT = 1000
local RUNS = 5
local MIN_RUN_SECONDS = 0.05

math.randomseed(42)

local function randomVector(scale: number): vector
    return vector.create(
        (math.random() * 2 - 1) * scale,
        (math.random() * 2 - 1) * scale,
        (math.random() * 2 - 1) * scale
    )
end

local points, velocities = {}, {}
for i = 1, POINT_COUNT do
    points[i] = randomVector(200)
    velocities[i] = randomVector(50)
end

local alphas, distanceFractions, editIndices, editPositions = {}, {}, {}, {}
for i = 1, 1024 do
    alphas[i] = math.random()
    distanceFractions[i] = math.random()
    editIndices[i] = math.random(1, POINT_COUNT)
    editPositions[i] = randomVector(200)
end

-- Path.new reads `.Position` from parts, so the baseline gets stand-in parts holding the same points
local waypointParts: { { Position: vector } } = {}

local function buildPath(): Path.Path
    waypointParts = table.create(POINT_COUNT)
    for i, point in points do
        waypointParts[i] = { Position = point }
    end
    return Path.new(1, waypointParts :: any)
end

type Column = {
    name: string,
    build: () -> any,
    -- Static splines can't be edited, so the edit rows skip them
    editable: boolean,
    -- Has corner radii, for the setRadius row
    corners: boolean?,
}

local COLUMNS: { Column } = {
    { name = "Bezier", editable = true, build = function() return Splynx.bezier(points) end },
    { name = "Hermite", editable = true, build = function() return Splynx.hermite(points, velocities) end },
    { name = "CatmullRom", editable = true, build = function() return Splynx.catmullRom(points) end },
    { name = "BSpline", editable = true, build = function() return Splynx.bSpline(points) end },
    { name = "Linear", editable = true, build = function() return Splynx.linear(points) end },
    {
        name = "Rounded",
        editable = true,
        corners = true,
        build = function()
            return Splynx.rounded(points, CORNER_SIZE)
        end,
    },
    {
        name = "Filleted",
        editable = true,
        corners = true,
        build = function()
            return Splynx.filleted(points, CORNER_SIZE)
        end,
    },
    {
        name = "Sampled",
        editable = false,
        build = function()
            return Splynx.sampled(Splynx.catmullRom(points), SAMPLED_SPACING)
        end,
    },
    {
        name = "SampledKept",
        editable = false,
        build = function()
            return Splynx.sampled(Splynx.rounded(points, CORNER_SIZE), SAMPLED_SPACING, true)
        end,
    },
}

-- Evenly spread units, each with its own distance, hint and offset
local unitDistances, unitOffsets = table.create(UNIT_COUNT), table.create(UNIT_COUNT)
for i = 1, UNIT_COUNT do
    unitDistances[i] = (i - 1) / UNIT_COUNT
    unitOffsets[i] = (i % 5) - 2
end

type Benchmark = {
    name: string,
    -- Runs `iterations` operations and returns how many units of work they were
    run: (spline: any, iterations: number) -> number,
    edits: boolean?,
    -- Only for splines with corner radii
    cornerEdits: boolean?,
    -- The same work done with the Path baseline, when it has an equivalent
    path: ((path: Path.Path, iterations: number) -> number)?,
}

local BENCHMARKS: { Benchmark } = {
    {
        name = "getPos",
        run = function(spline, iterations)
            for i = 1, iterations do
                spline:getPos(alphas[i % 1024 + 1])
            end
            return iterations
        end,
    },
    {
        name = `sample({BATCH_SIZE}) per point`,
        run = function(spline, iterations)
            for _ = 1, iterations do
                spline:sample(BATCH_SIZE)
            end
            return iterations * BATCH_SIZE
        end,
    },
    {
        name = "getPosAtDistance",
        run = function(spline, iterations)
            local length = spline:getLength()
            for i = 1, iterations do
                spline:getPosAtDistance(distanceFractions[i % 1024 + 1] * length)
            end
            return iterations
        end,
        -- MoveAlong needs a waypoint index to search from; random lookups pass the previous result's index
        path = function(path, iterations)
            local length = path.length
            local hint = 1
            for i = 1, iterations do
                local _, _, index = path:MoveAlong(distanceFractions[i % 1024 + 1] * length, hint)
                hint = index
            end
            return iterations
        end,
    },
    {
        -- Something moving along the spline: small steps forward, passing back the previous hint
        name = "getPosAtDistance moving + hint",
        run = function(spline, iterations)
            local length = spline:getLength()
            local step = length / 997
            local distance = 0
            local hint
            for _ = 1, iterations do
                distance = (distance + step) % length
                local _, newHint = spline:getPosAtDistance(distance, hint)
                hint = newHint
            end
            return iterations
        end,
        path = function(path, iterations)
            local length = path.length
            local step = length / 997
            local distance = 0
            local hint = 1
            for _ = 1, iterations do
                distance = (distance + step) % length
                local _, _, index = path:MoveAlong(distance, hint)
                hint = index
            end
            return iterations
        end,
    },
    {
        name = "getPosAndDirection moving + hint",
        run = function(spline, iterations)
            local length = spline:getLength()
            local step = length / 997
            local distance = 0
            local hint
            for _ = 1, iterations do
                distance = (distance + step) % length
                local _, _, newHint = spline:getPosAndDirectionAtDistance(distance, hint)
                hint = newHint
            end
            return iterations
        end,
        -- MoveAlong returns the direction as well
        path = function(path, iterations)
            local length = path.length
            local step = length / 997
            local distance = 0
            local hint = 1
            for _ = 1, iterations do
                distance = (distance + step) % length
                local _, _, index = path:MoveAlong(distance, hint)
                hint = index
            end
            return iterations
        end,
    },
    {
        name = "getOffsetPos moving + hint",
        run = function(spline, iterations)
            local length = spline:getLength()
            local step = length / 997
            local distance = 0
            local hint
            for _ = 1, iterations do
                distance = (distance + step) % length
                local _, _, newHint = spline:getOffsetPosAtDistance(distance, 1.5, hint)
                hint = newHint
            end
            return iterations
        end,
    },
    {
        -- Every unit steps forward, then one batch call fills positions and directions, reusing every list
        name = `getPosAtDistances({UNIT_COUNT}) per unit`,
        run = function(spline, iterations)
            local length = spline:getLength()
            local step = length / 997
            local distances = table.create(UNIT_COUNT)
            for i, fraction in unitDistances do
                distances[i] = fraction * length
            end
            local positions, directions, hints = table.create(UNIT_COUNT), table.create(UNIT_COUNT), {}
            for _ = 1, iterations do
                for i, distance in distances do
                    distances[i] = (distance + step) % length
                end
                spline:getPosAtDistances(distances, positions, directions, unitOffsets, hints)
            end
            return iterations * UNIT_COUNT
        end,
        -- The same units with MoveAlong, each keeping its own waypoint index
        path = function(path, iterations)
            local length = path.length
            local step = length / 997
            local distances = table.create(UNIT_COUNT)
            local indices = table.create(UNIT_COUNT, 1)
            for i, fraction in unitDistances do
                distances[i] = fraction * length
            end
            local positions, directions = table.create(UNIT_COUNT), table.create(UNIT_COUNT)
            for _ = 1, iterations do
                for i, distance in distances do
                    distances[i] = (distance + step) % length
                    local position, direction, index = path:MoveAlong(distances[i], indices[i])
                    positions[i], directions[i], indices[i] = position, direction, index
                end
            end
            return iterations * UNIT_COUNT
        end,
    },
    {
        name = `sampleUniform({BATCH_SIZE}) per point`,
        run = function(spline, iterations)
            for _ = 1, iterations do
                spline:sampleUniform(BATCH_SIZE)
            end
            return iterations * BATCH_SIZE
        end,
        -- Evenly spaced MoveAlong calls, carrying the index forward, into an output table like sampleUniform
        path = function(path, iterations)
            local step = path.length / (BATCH_SIZE - 1)
            for _ = 1, iterations do
                local out = table.create(BATCH_SIZE)
                local hint = 1
                for i = 0, BATCH_SIZE - 1 do
                    local position, _, index = path:MoveAlong(i * step, hint)
                    out[i + 1] = position
                    hint = index
                end
            end
            return iterations * BATCH_SIZE
        end,
    },
    {
        name = `sampleUniform({BATCH_SIZE}, out) per point`,
        run = function(spline, iterations)
            local out = table.create(BATCH_SIZE)
            for _ = 1, iterations do
                spline:sampleUniform(BATCH_SIZE, out)
            end
            return iterations * BATCH_SIZE
        end,
    },
    {
        name = "getLength",
        run = function(spline, iterations)
            for _ = 1, iterations do
                spline:getLength()
            end
            return iterations
        end,
        path = function(path, iterations)
            local length = 0
            for _ = 1, iterations do
                length = path.length
            end
            return iterations + length * 0
        end,
    },
    {
        name = "setPoint + getLength",
        edits = true,
        run = function(spline, iterations)
            for i = 1, iterations do
                local index = i % 1024 + 1
                spline:setPoint(editIndices[index], editPositions[index])
                spline:getLength()
            end
            return iterations
        end,
        -- Path can't be edited, so the equivalent is building a new one after moving a waypoint
        path = function(_path, iterations)
            for i = 1, iterations do
                local index = i % 1024 + 1
                waypointParts[editIndices[index]].Position = editPositions[index]
                Path.new(1, waypointParts :: any)
            end
            return iterations
        end,
    },
    {
        name = "setPoint + getPos",
        edits = true,
        run = function(spline, iterations)
            for i = 1, iterations do
                local index = i % 1024 + 1
                spline:setPoint(editIndices[index], editPositions[index])
                spline:getPos(alphas[index])
            end
            return iterations
        end,
    },
    {
        name = "setRadius + getLength",
        cornerEdits = true,
        run = function(spline, iterations)
            for i = 1, iterations do
                local index = i % 1024 + 1
                spline:setRadius(editIndices[index], alphas[index] * CORNER_SIZE * 2)
                spline:getLength()
            end
            return iterations
        end,
    },
    {
        name = `sampled(spacing {SAMPLED_SPACING})`,
        edits = true,
        run = function(spline, iterations)
            for _ = 1, iterations do
                Splynx.sampled(spline, SAMPLED_SPACING)
            end
            return iterations
        end,
    },
    {
        name = `sampled(spacing {SAMPLED_SPACING}, keepJoints)`,
        edits = true,
        run = function(spline, iterations)
            for _ = 1, iterations do
                Splynx.sampled(spline, SAMPLED_SPACING, true)
            end
            return iterations
        end,
    },
}

-- Grows the iteration count until a run takes long enough to time, then keeps the fastest of RUNS runs
local function measure(run: (subject: any, iterations: number) -> number, subject: any): number
    local iterations = 1
    while true do
        local start = os.clock()
        run(subject, iterations)
        if os.clock() - start >= MIN_RUN_SECONDS then
            break
        end
        iterations *= 2
    end

    local best = math.huge
    for _ = 1, RUNS do
        local start = os.clock()
        local units = run(subject, iterations)
        best = math.min(best, (os.clock() - start) / units)
    end
    return best * 1e9
end

local function formatNanoseconds(nanoseconds: number): string
    if nanoseconds >= 1e6 then
        return string.format("%.2f ms", nanoseconds / 1e6)
    elseif nanoseconds >= 1e3 then
        return string.format("%.2f us", nanoseconds / 1e3)
    end
    return string.format("%.1f ns", nanoseconds)
end

local NAME_WIDTH = 36
local COLUMN_WIDTH = 12

local function cell(text: string): string
    return string.format("%" .. COLUMN_WIDTH .. "s", text)
end

local header = string.rep(" ", NAME_WIDTH)
for _, column in COLUMNS do
    header ..= cell(column.name)
end
print(`{POINT_COUNT} control points; Rounded has a reach of {CORNER_SIZE} and Filleted a radius of {CORNER_SIZE}`)
print(`Sampled is the CatmullRom spline sampled every {SAMPLED_SPACING} studs; SampledKept is Rounded, keeping its joints`)
print(header .. cell("Path"))

for _, benchmark in BENCHMARKS do
    local row = string.format("%-" .. NAME_WIDTH .. "s", benchmark.name)
    for _, column in COLUMNS do
        if (benchmark.edits and not column.editable) or (benchmark.cornerEdits and not column.corners) then
            row ..= cell("-")
        else
            -- Fresh spline per cell so earlier edits don't leak between benchmarks
            row ..= cell(formatNanoseconds(measure(benchmark.run, column.build())))
        end
    end
    local pathRun = benchmark.path
    row ..= cell(if pathRun then formatNanoseconds(measure(pathRun, buildPath())) else "-")
    print(row)
end

print()
print("Path column: getPosAtDistance = MoveAlong (hint: previous index), moving + hint = MoveAlong in small steps")
print("(index carried forward), sampleUniform = MoveAlong at increasing distances (index carried forward),")
print("getLength = .length, setPoint + getLength = Path.new after moving a waypoint, getPosAtDistances = MoveAlong per")
print("unit (each keeping its own index), getPosAndDirection = MoveAlong, which returns the direction too.")
