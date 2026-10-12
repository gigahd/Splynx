--!native
--!optimize 2

local Vector = require("./Vector")

type VectorLike = Vector.VectorLike

export type Builder = {
    curveCount: (pointCount: number) -> number,
    affectedCurves: (pointIndex: number) -> (number, number),
    coefficients: (points: { vector }, index: number, velocities: { vector }?, scale: number) -> (vector, vector, vector, vector),
    usesVelocities: boolean?,
    usesScale: boolean?,
    velocity: ((points: { vector }, index: number, scale: number) -> vector)?,
}

local SEGMENTS = 32
local STRIDE = SEGMENTS + 1
local SEGMENT_WIDTH = 1 / SEGMENTS
local GAUSS_OFFSET = math.sqrt(3 / 5) * SEGMENT_WIDTH / 2
local GAUSS_SCALE = SEGMENT_WIDTH / 2 / 9
local BUCKETS_PER_CURVE = 4

local lengthBuffer = table.create(STRIDE, 0)
local speedBuffer = table.create(STRIDE, 0)

local NOT_ENOUGH_POINTS = "Spline doesn't have enough points to make a curve"

--[=[
    @class CubicSpline

    Smooth curves, made by [Splynx.bezier], [Splynx.hermite], [Splynx.cardinal], [Splynx.catmullRom] and
    [Splynx.bSpline]. Every type is stored the same way, as one cubic polynomial per curve, so they all cost the
    same to query; they only differ in how points become curves.

    Distance lookups use a table per curve that maps evenly spaced distances to alphas. It's built the first
    time a distance lookup or [CubicSpline:getLength] needs it, and rebuilt for the curves an edit changes, so
    an edit followed by a distance lookup costs a few microseconds. Positions are within about 0.0005 studs of
    the true distance.
]=]
local CubicSpline = {}
CubicSpline.__index = CubicSpline

type CubicSplineData = {
    builder: Builder,
    pivot: vector,
    points: { vector },
    velocities: { vector }?,
    scale: number,
    curveCount: number,
    c0: { vector },
    c1: { vector },
    c2: { vector },
    c3: { vector },
    curveLengths: { number },
    alphaTables: { number },
    alphaSlopes: { number },
    alphaScales: { number },
    tableBuilt: { boolean },
    cumulativeLengths: { number } | false,
    buckets: { number },
    bucketScale: number,
}

export type CubicSpline = setmetatable<CubicSplineData, typeof(CubicSpline)>

local function updateCurve(self: CubicSplineData, index: number)
    local c0, c1, c2, c3 = self.builder.coefficients(self.points, index, self.velocities, self.scale)
    self.c0[index] = c0
    self.c1[index] = c1
    self.c2[index] = c2
    self.c3[index] = c3
    self.tableBuilt[index] = false
end

local function updateCurves(self: CubicSplineData, first: number, last: number)
    for index = math.max(first, 1), math.min(last, self.curveCount) do
        updateCurve(self, index)
    end
    self.cumulativeLengths = false
end

local function reserveCurves(self: CubicSplineData, first: number, last: number)
    local alphas = self.alphaTables
    local slopes = self.alphaSlopes
    for i = (first - 1) * STRIDE + 1, last * STRIDE do
        alphas[i] = 0
        slopes[i] = 0
    end
    for index = first, last do
        self.curveLengths[index] = 0
        self.alphaScales[index] = 0
    end
end

local function rebuild(self: CubicSplineData)
    local curveCount = self.builder.curveCount(#self.points)
    self.curveCount = curveCount
    self.c0 = table.create(curveCount)
    self.c1 = table.create(curveCount)
    self.c2 = table.create(curveCount)
    self.c3 = table.create(curveCount)
    self.curveLengths = table.create(curveCount)
    self.alphaTables = table.create(curveCount * STRIDE)
    self.alphaSlopes = table.create(curveCount * STRIDE)
    self.alphaScales = table.create(curveCount)
    self.tableBuilt = table.create(curveCount, false)
    reserveCurves(self, 1, curveCount)
    updateCurves(self, 1, curveCount)
end

local function fitVelocities(self: CubicSplineData)
    local velocities = self.velocities
    if velocities then
        for i = #velocities + 1, #self.points do
            velocities[i] = vector.zero
        end
        for i = #velocities, #self.points + 1, -1 do
            velocities[i] = nil
        end
    end
end

local function invertLengths(curveDistance: number, segment: number): (number, number)
    local lengths = lengthBuffer
    while lengths[segment + 1] <= curveDistance do
        segment += 1
    end
    local segmentStart = lengths[segment]
    local segmentLength = lengths[segment + 1] - segmentStart
    if segmentLength <= 0 then
        return (segment - 1) * SEGMENT_WIDTH, segment
    end

    local target = curveDistance - segmentStart
    local startSlope = speedBuffer[segment] * SEGMENT_WIDTH
    local endSlope = speedBuffer[segment + 1] * SEGMENT_WIDTH
    local fraction = target / segmentLength
    for _ = 1, 3 do
        local f2 = fraction * fraction
        local f3 = f2 * fraction
        local difference = segmentLength * (3 * f2 - 2 * f3)
            + startSlope * (f3 - 2 * f2 + fraction)
            + endSlope * (f3 - f2)
            - target
        local slope = segmentLength * (6 * fraction - 6 * f2)
            + startSlope * (3 * f2 - 4 * fraction + 1)
            + endSlope * (3 * f2 - 2 * fraction)
        if slope <= 0 then
            break
        end
        fraction = math.clamp(fraction - difference / slope, 0, 1)
    end
    return (segment - 1 + fraction) * SEGMENT_WIDTH, segment
end

local function buildTables(self: CubicSplineData, index: number)
    local d0: vector = self.c1[index]
    local d1: vector = 2 * self.c2[index]
    local d2: vector = 3 * self.c3[index]

    local lengths = lengthBuffer
    local speeds = speedBuffer
    lengths[1] = 0
    speeds[1] = vector.magnitude(d0)
    local total = 0
    for segment = 1, SEGMENTS do
        local center = (segment - 0.5) * SEGMENT_WIDTH
        local before = center - GAUSS_OFFSET
        local after = center + GAUSS_OFFSET
        local finish = segment * SEGMENT_WIDTH
        total += GAUSS_SCALE * (
            5 * vector.magnitude(d0 + before * (d1 + before * d2))
            + 8 * vector.magnitude(d0 + center * (d1 + center * d2))
            + 5 * vector.magnitude(d0 + after * (d1 + after * d2))
        )
        lengths[segment + 1] = total
        speeds[segment + 1] = vector.magnitude(d0 + finish * (d1 + finish * d2))
    end
    self.curveLengths[index] = total
    self.alphaScales[index] = if total > 0 then SEGMENTS / total else 0

    local alphas = self.alphaTables
    local slopes = self.alphaSlopes
    local base = (index - 1) * STRIDE
    local step = total / SEGMENTS
    local segment = 1
    for entry = 0, SEGMENTS do
        local t
        if entry == 0 then
            t = 0
        elseif entry == SEGMENTS then
            t = 1
        else
            t, segment = invertLengths(entry * step, segment)
        end
        local speed = vector.magnitude(d0 + t * (d1 + t * d2))
        alphas[base + entry + 1] = t
        slopes[base + entry + 1] = if speed > 1e-6 then step / speed else 0
    end
    self.tableBuilt[index] = true
end

local function getCumulativeLengths(self: CubicSplineData): { number }
    local cumulativeLengths = self.cumulativeLengths
    if cumulativeLengths then
        return cumulativeLengths
    end

    local curveCount = self.curveCount
    local tableBuilt = self.tableBuilt
    local curveLengths = self.curveLengths
    local out = table.create(curveCount)
    local total = 0
    for index = 1, curveCount do
        if not tableBuilt[index] then
            buildTables(self, index)
        end
        total += curveLengths[index]
        out[index] = total
    end

    local bucketCount = curveCount * BUCKETS_PER_CURVE
    local bucketWidth = total / math.max(bucketCount, 1)
    local buckets = table.create(bucketCount)
    local index = 1
    for bucket = 1, bucketCount do
        local start = (bucket - 1) * bucketWidth
        while index < curveCount and out[index] < start do
            index += 1
        end
        buckets[bucket] = index
    end
    self.buckets = buckets
    self.bucketScale = if total > 0 then bucketCount / total else 0

    self.cumulativeLengths = out
    return out
end

local function evaluate(self: CubicSplineData, index: number, t: number): vector
    local c0: vector = self.c0[index]
    local c1: vector = self.c1[index]
    local c2: vector = self.c2[index]
    local c3: vector = self.c3[index]
    return self.pivot + c0 + t * (c1 + t * (c2 + t * c3))
end

local function locate(self: CubicSplineData, distance: number, hint: number?): (number, number)
    local curveCount = self.curveCount
    assert(curveCount > 0, NOT_ENOUGH_POINTS)
    local cumulativeLengths = self.cumulativeLengths or getCumulativeLengths(self)
    if distance <= 0 then
        return 1, 0
    elseif distance >= cumulativeLengths[curveCount] then
        return curveCount, 1
    end

    local index
    if hint then
        index = hint
        if index < 1 or index > curveCount or index % 1 ~= 0 then
            index = math.clamp(index // 1, 1, curveCount)
        end
    else
        local bucket = distance * self.bucketScale // 1 + 1
        if bucket > curveCount * BUCKETS_PER_CURVE then
            bucket = curveCount * BUCKETS_PER_CURVE
        end
        index = self.buckets[bucket]
    end
    while cumulativeLengths[index] < distance do
        index += 1
    end
    while index > 1 and cumulativeLengths[index - 1] >= distance do
        index -= 1
    end

    local position = (distance - (if index > 1 then cumulativeLengths[index - 1] else 0)) * self.alphaScales[index]
    local entry = position // 1
    if entry >= SEGMENTS then
        entry = SEGMENTS - 1
    end
    local f = position - entry
    local f2 = f * f
    local f3 = f2 * f
    local i = (index - 1) * STRIDE + entry + 1
    local alphas = self.alphaTables
    local slopes = self.alphaSlopes
    local start = alphas[i]
    local t = start
        + (alphas[i + 1] - start) * (3 * f2 - 2 * f3)
        + slopes[i] * (f3 - 2 * f2 + f)
        + slopes[i + 1] * (f3 - f2)
    return index, math.clamp(t, 0, 1)
end

function CubicSpline.new(
    builder: Builder,
    points: { VectorLike },
    pivot: VectorLike?,
    velocities: { VectorLike }?,
    scale: number?
): CubicSpline
    local data: CubicSplineData = {
        builder = builder,
        pivot = if pivot then Vector.from(pivot) else vector.zero,
        points = Vector.fromList(points),
        velocities = if builder.usesVelocities then Vector.fromList(velocities or {}) else nil,
        scale = scale or 0.5,
        curveCount = 0,
        c0 = {},
        c1 = {},
        c2 = {},
        c3 = {},
        curveLengths = {},
        alphaTables = {},
        alphaSlopes = {},
        alphaScales = {},
        tableBuilt = {},
        cumulativeLengths = false,
        buckets = {},
        bucketScale = 0,
    }
    local self = setmetatable(data, CubicSpline)
    fitVelocities(self)
    rebuild(self)
    return self
end

--[=[
    @within CubicSpline
    @method getPos
    @param alpha number -- From 0 (the start) to 1 (the end). Clamped.
    @return vector
    @error "Spline doesn't have enough points to make a curve" -- If the spline has no curves.

    The position at `alpha`. Alpha splits evenly across curves, so with 4 curves, 0.25 is the end of the first.
    Since curves have different lengths, equal alpha steps don't move at a constant speed; use
    [CubicSpline:getUniformPos] or [CubicSpline:getPosAtDistance] for that.
]=]
function CubicSpline.getPos(self: CubicSplineData, alpha: number): vector
    local curveCount = self.curveCount
    assert(curveCount > 0, NOT_ENOUGH_POINTS)
    local u = math.clamp(alpha, 0, 1) * curveCount
    local index = math.min(math.floor(u) + 1, curveCount)
    return evaluate(self, index, u - (index - 1))
end

--[=[
    @within CubicSpline
    @method sample
    @param count number
    @return {vector}
    @error "Spline doesn't have enough points to make a curve" -- If the spline has no curves.

    `count` positions at evenly spaced alphas, including both ends. Faster than calling [CubicSpline:getPos] in a
    loop. Use [CubicSpline:sampleUniform] for positions evenly spaced along the length instead.
]=]
function CubicSpline.sample(self: CubicSplineData, count: number): { vector }
    if count <= 1 then
        return if count == 1 then { CubicSpline.getPos(self, 0) } else {}
    end
    local curveCount = self.curveCount
    assert(curveCount > 0, NOT_ENOUGH_POINTS)
    local step = curveCount / (count - 1)
    local out = table.create(count)
    for i = 0, count - 1 do
        local u = i * step
        local index = math.min(math.floor(u) + 1, curveCount)
        out[i + 1] = evaluate(self, index, u - (index - 1))
    end
    return out
end

--[=[
    @within CubicSpline
    @method getLength
    @return number

    The length along the spline, or 0 if it has no curves.
]=]
function CubicSpline.getLength(self: CubicSplineData): number
    local curveCount = self.curveCount
    if curveCount == 0 then
        return 0
    end
    return (self.cumulativeLengths or getCumulativeLengths(self))[curveCount]
end

--[=[
    @within CubicSpline
    @method getAlphaAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return number -- The alpha, as used by [CubicSpline:getPos].
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a curve" -- If the spline has no curves.

    The alpha at which `distance` along the spline has been travelled.
]=]
function CubicSpline.getAlphaAtDistance(self: CubicSplineData, distance: number, hint: number?): (number, number)
    local index, t = locate(self, distance, hint)
    return (index - 1 + t) / self.curveCount, index
end

--[=[
    @within CubicSpline
    @method getPosAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a curve" -- If the spline has no curves.

    The position `distance` studs along the spline. This is the call to use for things moving at a constant
    speed: keep a distance, add `speed * deltaTime` to it every frame, and pass back the hint from the last call.

    ```lua
    local distance, hint = 0, nil
    RunService.Heartbeat:Connect(function(deltaTime)
        distance = (distance + speed * deltaTime) % spline:getLength()
        local position
        position, hint = spline:getPosAtDistance(distance, hint)
    end)
    ```
]=]
function CubicSpline.getPosAtDistance(self: CubicSplineData, distance: number, hint: number?): (vector, number)
    local curveCount = self.curveCount
    assert(curveCount > 0, NOT_ENOUGH_POINTS)
    local cumulativeLengths = self.cumulativeLengths or getCumulativeLengths(self)
    if distance <= 0 then
        return evaluate(self, 1, 0), 1
    elseif distance >= cumulativeLengths[curveCount] then
        return evaluate(self, curveCount, 1), curveCount
    end

    local index
    if hint then
        index = hint
        if index < 1 or index > curveCount or index % 1 ~= 0 then
            index = math.clamp(index // 1, 1, curveCount)
        end
    else
        local bucket = distance * self.bucketScale // 1 + 1
        if bucket > curveCount * BUCKETS_PER_CURVE then
            bucket = curveCount * BUCKETS_PER_CURVE
        end
        index = self.buckets[bucket]
    end
    while cumulativeLengths[index] < distance do
        index += 1
    end
    while index > 1 and cumulativeLengths[index - 1] >= distance do
        index -= 1
    end

    local position = (distance - (if index > 1 then cumulativeLengths[index - 1] else 0)) * self.alphaScales[index]
    local entry = position // 1
    if entry >= SEGMENTS then
        entry = SEGMENTS - 1
    end
    local f = position - entry
    local f2 = f * f
    local f3 = f2 * f
    local i = (index - 1) * STRIDE + entry + 1
    local alphas = self.alphaTables
    local slopes = self.alphaSlopes
    local start = alphas[i]
    local t = start
        + (alphas[i + 1] - start) * (3 * f2 - 2 * f3)
        + slopes[i] * (f3 - 2 * f2 + f)
        + slopes[i + 1] * (f3 - f2)
    return evaluate(self, index, math.clamp(t, 0, 1)), index
end

--[=[
    @within CubicSpline
    @method getUniformPos
    @param alpha number -- From 0 (the start) to 1 (the end), as a fraction of the length. Clamped.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a curve" -- If the spline has no curves.

    Like [CubicSpline:getPos], but alpha is a fraction of the length, so equal alpha steps move at a constant
    speed. The same as [CubicSpline:getPosAtDistance] with `alpha * length`.
]=]
function CubicSpline.getUniformPos(self: CubicSplineData, alpha: number, hint: number?): (vector, number)
    return CubicSpline.getPosAtDistance(self, math.clamp(alpha, 0, 1) * CubicSpline.getLength(self), hint)
end

--[=[
    @within CubicSpline
    @method sampleUniform
    @param count number
    @return {vector}
    @error "Spline doesn't have enough points to make a curve" -- If the spline has no curves.

    `count` positions evenly spaced along the length, including both ends. Faster than calling
    [CubicSpline:getPosAtDistance] in a loop.
]=]
function CubicSpline.sampleUniform(self: CubicSplineData, count: number): { vector }
    if count <= 1 then
        return if count == 1 then { CubicSpline.getPos(self, 0) } else {}
    end
    local step = CubicSpline.getLength(self) / (count - 1)
    local out = table.create(count)
    local hint = 1
    for i = 0, count - 1 do
        local index, t = locate(self, i * step, hint)
        hint = index
        out[i + 1] = evaluate(self, index, t)
    end
    return out
end

--[=[
    @within CubicSpline
    @method getJoints
    @return {vector}

    Where each curve starts, plus where the last one ends. For Bezier splines these are every third point; for
    B-splines they're between the points.
]=]
function CubicSpline.getJoints(self: CubicSplineData): { vector }
    local curveCount = self.curveCount
    if curveCount == 0 then
        return {}
    end
    local out = table.create(curveCount + 1)
    for index = 1, curveCount do
        out[index] = evaluate(self, index, 0)
    end
    out[curveCount + 1] = evaluate(self, curveCount, 1)
    return out
end

--[=[
    @within CubicSpline
    @method getCurveCount
    @return number

    The number of curves the points make.
]=]
function CubicSpline.getCurveCount(self: CubicSplineData): number
    return self.curveCount
end

--[=[
    @within CubicSpline
    @method getPoint
    @param index number
    @return vector -- Relative to the pivot.
]=]
function CubicSpline.getPoint(self: CubicSplineData, index: number): vector
    return self.points[index]
end

--[=[
    @within CubicSpline
    @method getPoints
    @return {vector} -- Relative to the pivot.

    A copy of the points. Change them through [CubicSpline:setPoint] and friends so the spline stays up to date.
]=]
function CubicSpline.getPoints(self: CubicSplineData): { vector }
    return table.clone(self.points)
end

--[=[
    @within CubicSpline
    @method setPoint
    @param index number
    @param position VectorLike -- Relative to the pivot.
    @error "No point at index ..." -- If there's no point at `index`.

    Moves one point. Only the curves that use it are rebuilt.
]=]
function CubicSpline.setPoint(self: CubicSplineData, index: number, position: VectorLike)
    assert(self.points[index], `No point at index {index}`)
    self.points[index] = Vector.from(position)
    updateCurves(self, self.builder.affectedCurves(index))
end

--[=[
    @within CubicSpline
    @method setPoints
    @param points {VectorLike} -- Relative to the pivot.

    Replaces every point. Hermite splines keep their velocities in order, adding zero velocities for new points.
]=]
function CubicSpline.setPoints(self: CubicSplineData, points: { VectorLike })
    self.points = Vector.fromList(points)
    fitVelocities(self)
    rebuild(self)
end

--[=[
    @within CubicSpline
    @method addPoints
    @param points {VectorLike} -- Relative to the pivot.
    @param velocities {VectorLike}? -- Hermite splines only: one per added point. Missing ones are zero.

    Adds points to the end. Existing curves keep their tables unless the new points change them.
]=]
function CubicSpline.addPoints(self: CubicSplineData, points: { VectorLike }, velocities: { VectorLike }?)
    local splinePoints = self.points
    local firstNew = #splinePoints + 1
    for i, position in points do
        splinePoints[firstNew + i - 1] = Vector.from(position)
    end
    local splineVelocities = self.velocities
    if splineVelocities and velocities then
        for i, velocity in velocities do
            splineVelocities[firstNew + i - 1] = Vector.from(velocity)
        end
    end
    fitVelocities(self)

    local oldCurveCount = self.curveCount
    local curveCount = self.builder.curveCount(#splinePoints)
    reserveCurves(self, oldCurveCount + 1, curveCount)
    self.curveCount = curveCount
    local first = self.builder.affectedCurves(firstNew)
    updateCurves(self, first, curveCount)
end

--[=[
    @within CubicSpline
    @method removePoints
    @param indices {number} -- In any order. Repeated indices are removed once.

    Removes several points at once.
]=]
function CubicSpline.removePoints(self: CubicSplineData, indices: { number })
    local sorted = table.clone(indices)
    table.sort(sorted, function(a, b)
        return a > b
    end)
    local previous
    for _, index in sorted do
        if index ~= previous then
            table.remove(self.points, index)
            if self.velocities then
                table.remove(self.velocities, index)
            end
            previous = index
        end
    end
    rebuild(self)
end

--[=[
    @within CubicSpline
    @method removePoint
    @param index number
]=]
function CubicSpline.removePoint(self: CubicSplineData, index: number)
    CubicSpline.removePoints(self, { index })
end

--[=[
    @within CubicSpline
    @method getPivot
    @return vector
]=]
function CubicSpline.getPivot(self: CubicSplineData): vector
    return self.pivot
end

--[=[
    @within CubicSpline
    @method setPivot
    @param pivot VectorLike

    Moves the whole spline. Points stay the same relative to the pivot, so nothing needs rebuilding.
]=]
function CubicSpline.setPivot(self: CubicSplineData, pivot: VectorLike)
    self.pivot = Vector.from(pivot)
end

--[=[
    @within CubicSpline
    @method getVelocity
    @param index number
    @return vector
    @error "Only Hermite and Cardinal splines have velocities" -- On other spline types.

    The velocity at a point: a Hermite spline's own, or the one a Cardinal spline derives from the neighbours.
]=]
function CubicSpline.getVelocity(self: CubicSplineData, index: number): vector
    local velocities = self.velocities
    if velocities then
        return velocities[index]
    end
    local velocity = self.builder.velocity
    assert(velocity, "Only Hermite and Cardinal splines have velocities")
    return velocity(self.points, index, self.scale)
end

--[=[
    @within CubicSpline
    @method setVelocity
    @param index number
    @param velocity VectorLike
    @error "Only Hermite splines have settable velocities" -- On other spline types.
    @error "No point at index ..." -- If there's no point at `index`.

    Changes a Hermite spline's velocity at a point. Only the curves either side of it are rebuilt.
]=]
function CubicSpline.setVelocity(self: CubicSplineData, index: number, velocity: VectorLike)
    local velocities = self.velocities
    assert(velocities, "Only Hermite splines have settable velocities")
    assert(velocities[index], `No point at index {index}`)
    velocities[index] = Vector.from(velocity)
    updateCurves(self, self.builder.affectedCurves(index))
end

--[=[
    @within CubicSpline
    @method getScale
    @return number

    A Cardinal spline's scale.
]=]
function CubicSpline.getScale(self: CubicSplineData): number
    return self.scale
end

--[=[
    @within CubicSpline
    @method setScale
    @param scale number
    @error "Only Cardinal splines have a scale" -- On other spline types.

    Changes a Cardinal spline's scale, which reshapes every curve.
]=]
function CubicSpline.setScale(self: CubicSplineData, scale: number)
    assert(self.builder.usesScale, "Only Cardinal splines have a scale")
    self.scale = scale
    updateCurves(self, 1, self.curveCount)
end

return CubicSpline
