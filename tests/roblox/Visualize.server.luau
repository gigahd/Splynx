-- Draws one row per spline type and keeps each row in sync with its parts.
-- Start a Run session (F8), then drag the spheres (control points) or a row's grey cube (pivot) with the Move tool.
-- White cubes are curve joints and thin lines from points are velocities (drawn at 1/3 length).
-- The black ball moves at TRAVEL_SPEED studs/s with getPosAtDistance (distance += dt * speed), and the label shows
-- its measured speed. The white ball uses getPos(distance / length): it finishes each lap at the same time but speeds
-- up and slows down, and any jump at a joint is easy to spot. The cube walks SIDE_OFFSET studs to the right of the
-- black ball with getOffsetPosAtDistance, facing the direction of travel. Behind it, UNIT_COUNT smaller cubes walk
-- in pairs either side of the spline, all placed by one getPosAtDistances call per frame.
-- Rounded and Filleted rows have a CornerSize attribute on their pivot: change it in the Properties window to set
-- every corner's reach (Rounded) or radius (Filleted) and watch the difference at sharp and shallow corners.
-- Every spline type is followed by a sampled row: the same spline sampled every SAMPLED_SPACING studs, drawn as its
-- sampled segments with a marker at each sample. It samples again when you move a point. Both balls move at a
-- constant speed there, since a sampled spline's alpha is already uniform. Linear, Rounded and Filleted keep their
-- joints when sampled, so the corners stay where they are.
-- Moving parts from the server looks choppy in Play mode (F5), so use Run mode (F8) to judge smoothness.

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

local Splynx = require(ReplicatedStorage:WaitForChild("Splynx"))

type Spline = Splynx.Spline
type Source = Splynx.LinearSpline | Splynx.CubicSpline

local SAMPLES_PER_CURVE = 24
local ROW_SPACING = 40
local TRAVEL_SPEED = 12 -- studs per second
local CURVE_THICKNESS = 0.35
local GUIDE_THICKNESS = 0.12
local SAMPLED_SPACING = 0.5
local SIDE_OFFSET = 2
local CORNER_SIZE = 4
local UNIT_COUNT = 8
local UNIT_GAP = 2.5 -- studs between pairs of units along the spline

local POINTS = {
    Vector3.new(0, 0, 0),
    Vector3.new(10, 12, 0),
    Vector3.new(20, -6, 0),
    Vector3.new(30, 4, 0),
    Vector3.new(40, 14, 0),
    Vector3.new(50, -4, 0),
    Vector3.new(60, 6, 0),
}
local VELOCITIES = {
    Vector3.new(15, 30, 0),
    Vector3.new(20, 0, 0),
    Vector3.new(15, -30, 0),
    Vector3.new(10, 15, 0),
    Vector3.new(30, 0, 0),
    Vector3.new(15, -15, 0),
    Vector3.new(15, 30, 0),
}

type DemoDefinition = {
    name: string,
    build: (points: { Vector3 }, pivot: Vector3) -> Source,
    showVelocities: boolean?,
    showControlPolygon: boolean?,
    -- Put a sample on every joint when sampling
    keepJoints: boolean?,
    -- Starting corner size, editable through the pivot's CornerSize attribute, and what it measures
    cornerSize: number?,
    cornerMeasure: string?,
    -- Draw and travel along a sampled copy of the spline instead
    sampledSpacing: number?,
}

local SPLINE_DEFINITIONS: { DemoDefinition } = {
    {
        name = "Bezier",
        build = function(points, pivot)
            return Splynx.bezier(points, pivot)
        end,
        showControlPolygon = true,
    },
    {
        name = "Hermite",
        build = function(points, pivot)
            return Splynx.hermite(points, VELOCITIES, pivot)
        end,
        showVelocities = true,
    },
    {
        name = "Catmull-Rom",
        build = function(points, pivot)
            return Splynx.catmullRom(points, pivot)
        end,
        showVelocities = true,
    },
    {
        name = "Cardinal (scale 1)",
        build = function(points, pivot)
            return Splynx.cardinal(points, 1, pivot)
        end,
        showVelocities = true,
    },
    {
        name = "Linear",
        build = function(points, pivot)
            return Splynx.linear(points, pivot)
        end,
        keepJoints = true,
    },
    {
        name = "Rounded",
        build = function(points, pivot)
            return Splynx.rounded(points, CORNER_SIZE, pivot)
        end,
        showControlPolygon = true,
        keepJoints = true,
        cornerSize = CORNER_SIZE,
        cornerMeasure = "reach",
    },
    {
        name = "Filleted",
        build = function(points, pivot)
            return Splynx.filleted(points, CORNER_SIZE, pivot)
        end,
        showControlPolygon = true,
        keepJoints = true,
        cornerSize = CORNER_SIZE,
        cornerMeasure = "radius",
    },
    {
        name = "B-Spline",
        build = function(points, pivot)
            return Splynx.bSpline(points, pivot)
        end,
        showControlPolygon = true,
    },
}

-- Each spline type, then a sampled copy of it
local DEFINITIONS: { DemoDefinition } = {}
for _, definition in SPLINE_DEFINITIONS do
    table.insert(DEFINITIONS, definition)
    local keeping = if definition.keepJoints then ", keeping joints" else ""
    table.insert(DEFINITIONS, {
        name = `{definition.name} sampled every {SAMPLED_SPACING} studs{keeping}`,
        build = definition.build,
        keepJoints = definition.keepJoints,
        cornerSize = definition.cornerSize,
        cornerMeasure = definition.cornerMeasure,
        sampledSpacing = SAMPLED_SPACING,
    })
end

-- Splynx returns vectors, which are Vector3s at runtime on Roblox; this only satisfies the type checker
local function toVector3(value: vector): Vector3
    return (value :: any) :: Vector3
end

local function makePart(props: { [string]: any }): Part
    local part = Instance.new("Part")
    part.Anchored = true
    part.CanCollide = false
    part.CanTouch = false
    part.CastShadow = false
    part.Material = Enum.Material.SmoothPlastic
    for key, value in props do
        (part :: any)[key] = value
    end
    return part
end

-- Drawn geometry is locked and unqueryable so clicks go through to the editable parts
local function makeDecoration(color: Color3, material: Enum.Material?): Part
    return makePart({
        Color = color,
        Material = material or Enum.Material.Neon,
        Locked = true,
        CanQuery = false,
    })
end

type Pool = {
    folder: Folder,
    parts: { Part },
    create: () -> Part,
}

local function newPool(name: string, parent: Instance, create: () -> Part): Pool
    local folder = Instance.new("Folder")
    folder.Name = name
    folder.Parent = parent
    return { folder = folder, parts = {}, create = create }
end

local function acquire(pool: Pool, index: number): Part
    local part = pool.parts[index]
    if not part then
        part = pool.create()
        part.Parent = pool.folder
        pool.parts[index] = part
    end
    return part
end

local function trim(pool: Pool, used: number)
    for i = #pool.parts, used + 1, -1 do
        pool.parts[i]:Destroy()
        pool.parts[i] = nil
    end
end

local function placeSegment(part: Part, from: Vector3, to: Vector3, thickness: number)
    local length = (to - from).Magnitude
    part.Size = Vector3.new(thickness, thickness, math.max(length, 0.01))
    part.CFrame = if length > 1e-4 then CFrame.lookAt((from + to) / 2, to) else CFrame.new(from)
end

local function drawPolyline(pool: Pool, positions: { Vector3 }, thickness: number)
    for i = 1, #positions - 1 do
        placeSegment(acquire(pool, i), positions[i], positions[i + 1], thickness)
    end
    trim(pool, math.max(#positions - 1, 0))
end

type Demo = {
    definition: DemoDefinition,
    -- The editable spline the parts control
    source: Source,
    -- What gets drawn and travelled along: the source, or a sampled copy of it
    spline: Spline,
    pivotPart: Part,
    pointParts: { Part },
    lastPivot: Vector3,
    lastPoints: { Vector3 },
    curvePool: Pool,
    jointPool: Pool,
    guidePool: Pool,
    traveler: Part,
    uniformTraveler: Part,
    sideTraveler: Part,
    -- The units walking either side of the spline, and the lists getPosAtDistances reads and fills each frame
    units: { Part },
    unitDistances: { number },
    unitOffsets: { number },
    unitPositions: { vector },
    unitDirections: { vector },
    unitHints: { number },
    cornerSize: number?,
    label: TextLabel,
    summary: string,
    distance: number,
    hint: number?,
}

local root = Instance.new("Folder")
root.Name = "SplynxDemo"

-- The source's points with the pivot applied
local function globalPoints(source: Source): { Vector3 }
    local pivot = toVector3(source:getPivot())
    local out = {}
    for i, point in source:getPoints() do
        out[i] = pivot + toVector3(point)
    end
    return out
end

local function redraw(demo: Demo)
    local source = demo.source
    local spacing = demo.definition.sampledSpacing

    local curve, joints = {}, {}
    local jointSize
    local name = demo.definition.name
    local cornerSize = demo.cornerSize
    if cornerSize then
        name = string.format("%s (%s %.2f)", name, demo.definition.cornerMeasure or "size", cornerSize)
    end
    if spacing then
        -- The sampled segments themselves, with a marker at every sample
        local sampled = demo.spline :: Splynx.SampledSpline
        local pivot = toVector3(sampled:getPivot())
        for i, point in sampled:getPoints() do
            curve[i] = pivot + toVector3(point)
        end
        joints = curve
        jointSize = 0.5
        demo.summary = string.format("%s\n%d segments, length %.2f", name, sampled:getCurveCount(), sampled:getLength())
    else
        for i, position in source:sample(source:getCurveCount() * SAMPLES_PER_CURVE + 1) do
            curve[i] = toVector3(position)
        end
        for i, joint in source:getJoints() do
            joints[i] = toVector3(joint)
        end
        jointSize = 0.9
        demo.summary = string.format("%s\n%d curves, length %.2f", name, source:getCurveCount(), source:getLength())
    end
    drawPolyline(demo.curvePool, curve, CURVE_THICKNESS)

    for i, joint in joints do
        local part = acquire(demo.jointPool, i)
        part.Size = Vector3.one * jointSize
        part.Position = joint
    end
    trim(demo.jointPool, #joints)

    local guideCount = 0
    if demo.definition.showControlPolygon then
        local positions = globalPoints(source)
        for i = 1, #positions - 1 do
            guideCount += 1
            placeSegment(acquire(demo.guidePool, guideCount), positions[i], positions[i + 1], GUIDE_THICKNESS)
        end
    end
    if demo.definition.showVelocities then
        local cubic = source :: Splynx.CubicSpline
        for i, from in globalPoints(source) do
            guideCount += 1
            local velocity = toVector3(cubic:getVelocity(i))
            placeSegment(acquire(demo.guidePool, guideCount), from, from + velocity / 3, GUIDE_THICKNESS)
        end
    end
    trim(demo.guidePool, guideCount)
end

local function createDemo(definition: DemoDefinition, row: number): Demo
    -- A spline and its sampled copy share a colour
    local color = Color3.fromHSV(((row - 1) // 2) / #SPLINE_DEFINITIONS, 0.75, 1)
    local origin = Vector3.new(0, 15, (row - 1) * ROW_SPACING)

    local folder = Instance.new("Folder")
    folder.Name = definition.name
    folder.Parent = root

    local source = definition.build(POINTS, origin)
    local spacing = definition.sampledSpacing
    local spline: Spline = if spacing then Splynx.sampled(source, spacing, definition.keepJoints) else source

    local pivotPart = makePart({
        Name = "Pivot",
        Size = Vector3.one * 1.5,
        Color = Color3.fromRGB(120, 120, 120),
        Position = origin,
        Parent = folder,
    })

    local billboard = Instance.new("BillboardGui")
    billboard.Size = UDim2.fromOffset(220, 70)
    billboard.StudsOffset = Vector3.new(0, 3, 0)
    billboard.AlwaysOnTop = true
    billboard.Parent = pivotPart
    local label = Instance.new("TextLabel")
    label.Size = UDim2.fromScale(1, 1)
    label.BackgroundTransparency = 1
    label.TextColor3 = Color3.new(1, 1, 1)
    label.TextStrokeTransparency = 0.3
    label.Font = Enum.Font.GothamBold
    label.TextSize = 16
    label.Parent = billboard

    if definition.cornerSize then
        pivotPart:SetAttribute("CornerSize", definition.cornerSize)
    end

    local units, unitDistances, unitOffsets = {}, {}, {}
    for i = 1, UNIT_COUNT do
        local unit = makeDecoration(Color3.fromRGB(60, 60, 60), Enum.Material.SmoothPlastic)
        unit.Size = Vector3.new(0.6, 0.6, 1)
        unit.Name = `Unit{i}`
        unit.Parent = folder
        units[i] = unit
        unitDistances[i] = 0
        -- Pairs, one either side
        unitOffsets[i] = if i % 2 == 1 then SIDE_OFFSET else -SIDE_OFFSET
    end

    local pointParts = {}
    local lastPoints = {}
    for i, position in globalPoints(source) do
        pointParts[i] = makePart({
            Name = `P{i}`,
            Shape = Enum.PartType.Ball,
            Size = Vector3.one * 1.4,
            Color = color,
            Position = position,
            Parent = folder,
        })
        lastPoints[i] = pointParts[i].Position
    end

    local demo: Demo = {
        definition = definition,
        source = source,
        spline = spline,
        pivotPart = pivotPart,
        pointParts = pointParts,
        lastPivot = pivotPart.Position,
        lastPoints = lastPoints,
        curvePool = newPool("Curve", folder, function()
            return makeDecoration(color)
        end),
        jointPool = newPool("Joints", folder, function()
            return makeDecoration(Color3.new(1, 1, 1))
        end),
        guidePool = newPool("Guides", folder, function()
            return makeDecoration(Color3.fromRGB(170, 170, 170), Enum.Material.SmoothPlastic)
        end),
        traveler = makeDecoration(Color3.new(1, 1, 1)),
        uniformTraveler = makeDecoration(Color3.new(0, 0, 0), Enum.Material.SmoothPlastic),
        sideTraveler = makeDecoration(Color3.new(0, 0, 0), Enum.Material.SmoothPlastic),
        units = units,
        unitDistances = unitDistances,
        unitOffsets = unitOffsets,
        unitPositions = table.create(UNIT_COUNT),
        unitDirections = table.create(UNIT_COUNT),
        unitHints = table.create(UNIT_COUNT),
        cornerSize = definition.cornerSize,
        label = label,
        summary = "",
        distance = 0,
        hint = nil,
    }
    demo.traveler.Shape = Enum.PartType.Ball
    demo.traveler.Size = Vector3.one * 1.8
    demo.traveler.Name = "Traveler"
    demo.traveler.Parent = folder
    demo.uniformTraveler.Shape = Enum.PartType.Ball
    demo.uniformTraveler.Size = Vector3.one * 1.8
    demo.uniformTraveler.Name = "UniformTraveler"
    demo.uniformTraveler.Parent = folder
    demo.sideTraveler.Size = Vector3.new(0.8, 0.8, 1.6)
    demo.sideTraveler.Name = "SideTraveler"
    demo.sideTraveler.Parent = folder

    redraw(demo)
    return demo
end

-- Pulls part edits into the spline. Moving the pivot drags every point part along with it.
local function sync(demo: Demo): boolean
    local source = demo.source
    local changed = false

    if demo.pivotPart.Position ~= demo.lastPivot then
        demo.lastPivot = demo.pivotPart.Position
        source:setPivot(demo.lastPivot)
        -- A sampled copy only needs moving, not sampling again
        demo.spline:setPivot(demo.lastPivot)
        for i, position in globalPoints(source) do
            demo.pointParts[i].Position = position
            demo.lastPoints[i] = position
        end
        changed = true
    end

    local pointsChanged = false
    for i, part in demo.pointParts do
        if part.Position ~= demo.lastPoints[i] then
            demo.lastPoints[i] = part.Position
            -- Points are relative to the pivot
            source:setPoint(i, part.Position - demo.lastPivot)
            pointsChanged = true
        end
    end

    local cornerSize = demo.pivotPart:GetAttribute("CornerSize")
    if demo.cornerSize and type(cornerSize) == "number" and cornerSize ~= demo.cornerSize then
        demo.cornerSize = cornerSize
        local cubic = source :: Splynx.CubicSpline
        for i in demo.pointParts do
            cubic:setRadius(i, cornerSize)
        end
        pointsChanged = true
    end

    local spacing = demo.definition.sampledSpacing
    if pointsChanged and spacing then
        demo.spline = Splynx.sampled(source, spacing, demo.definition.keepJoints)
    end

    return changed or pointsChanged
end

local demos = {}
for row, definition in DEFINITIONS do
    table.insert(demos, createDemo(definition, row))
end
root.Parent = workspace

RunService.Heartbeat:Connect(function(deltaTime)
    for _, demo in demos do
        if sync(demo) then
            redraw(demo)
        end

        local spline = demo.spline
        local length = spline:getLength()
        if length <= 0 then
            continue
        end
        demo.distance = (demo.distance + deltaTime * TRAVEL_SPEED) % length

        local previous = demo.uniformTraveler.Position
        local position, hint = spline:getPosAtDistance(demo.distance, demo.hint)
        demo.hint = hint
        demo.uniformTraveler.Position = toVector3(position)
        demo.traveler.Position = toVector3(spline:getPos(demo.distance / length))
        local sidePosition, direction = spline:getOffsetPosAtDistance(demo.distance, SIDE_OFFSET, hint)
        demo.sideTraveler.CFrame = CFrame.lookAt(toVector3(sidePosition), toVector3(sidePosition + direction))

        -- The units trail the black ball in pairs, and one call places them all, reusing the same lists every frame
        for i in demo.units do
            demo.unitDistances[i] = (demo.distance - ((i + 1) // 2) * UNIT_GAP) % length
        end
        spline:getPosAtDistances(
            demo.unitDistances,
            demo.unitPositions,
            demo.unitDirections,
            demo.unitOffsets,
            demo.unitHints
        )
        for i, unit in demo.units do
            local unitPosition = toVector3(demo.unitPositions[i])
            unit.CFrame = CFrame.lookAt(unitPosition, unitPosition + toVector3(demo.unitDirections[i]))
        end

        -- Straight-line speed since the last frame. It dips for a frame when the ball wraps around or cuts a sharp
        -- corner (Linear), because the ball then travels further than the straight line between its two positions.
        local measured = (demo.uniformTraveler.Position - previous).Magnitude / deltaTime
        demo.label.Text = string.format("%s\nblack ball: %.1f studs/s", demo.summary, measured)
    end
end)
