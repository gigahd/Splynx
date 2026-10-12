--!native
--!optimize 2

local Vector = require("./Vector")

type VectorLike = Vector.VectorLike

local BUCKETS_PER_SEGMENT = 4

local NOT_ENOUGH_POINTS = "Spline doesn't have enough points to make a segment"

--[=[
    @class LinearSpline

    Straight segments between points, made by [Splynx.linear]. Each segment's direction and length are stored,
    so a distance lookup is a short walk plus one multiply-add.

    Here a "curve" is a segment: [LinearSpline:getCurveCount] counts segments, and [LinearSpline:getJoints] are
    the points.
]=]
local LinearSpline = {}
LinearSpline.__index = LinearSpline

type LinearSplineData = {
    pivot: vector,
    points: { vector },
    directions: { vector },
    -- Each segment's right, and the same with vertical segments borrowing a neighbour's, for offsets
    segmentSides: { vector },
    sides: { vector },
    segmentLengths: { number },
    distances: { number } | false,
    cachedLength: number,
    buckets: { number },
    bucketScale: number,
}

export type LinearSpline = setmetatable<LinearSplineData, typeof(LinearSpline)>

local function updateSegment(self: LinearSplineData, index: number)
    local delta = self.points[index + 1] - self.points[index]
    local length = vector.magnitude(delta)
    local direction = if length > 0 then delta / length else vector.zero
    self.directions[index] = direction
    self.segmentSides[index] = Vector.rightOf(direction)
    self.segmentLengths[index] = length
end

local function updateSegments(self: LinearSplineData, first: number, last: number)
    for index = math.max(first, 1), math.min(last, #self.points - 1) do
        updateSegment(self, index)
    end
    self.distances = false
end

local function rebuild(self: LinearSplineData)
    local segmentCount = math.max(#self.points - 1, 0)
    self.directions = table.create(segmentCount)
    self.segmentSides = table.create(segmentCount)
    self.segmentLengths = table.create(segmentCount)
    updateSegments(self, 1, segmentCount)
end

local function getDistances(self: LinearSplineData): { number }
    local distances = self.distances
    if distances then
        return distances
    end
    local pointCount = #self.points
    local segmentLengths = self.segmentLengths
    local out = table.create(math.max(pointCount, 1))
    out[1] = 0
    for index = 1, pointCount - 1 do
        out[index + 1] = out[index] + segmentLengths[index]
    end
    local total = out[#out]
    local segmentCount = pointCount - 1
    local bucketCount = math.max(segmentCount, 0) * BUCKETS_PER_SEGMENT
    local bucketWidth = total / math.max(bucketCount, 1)
    local buckets = table.create(bucketCount)
    local index = 1
    for bucket = 1, bucketCount do
        local start = (bucket - 1) * bucketWidth
        while index < segmentCount and out[index + 1] <= start do
            index += 1
        end
        buckets[bucket] = index
    end
    self.buckets = buckets
    self.bucketScale = if total > 0 then bucketCount / total else 0

    local sides = table.clone(self.segmentSides)
    Vector.fillGaps(sides)
    self.sides = sides

    self.distances = out
    self.cachedLength = total
    return out
end

-- The segment `distance` is on and how far along it. getPosAtDistance, getPosAndDirectionAtDistance,
-- getOffsetPosAtDistance and getPosAtDistances repeat this inline because it's faster, so change them together.
local function locate(self: LinearSplineData, distance: number, hint: number?): (number, number)
    local distances = self.distances or getDistances(self)
    local segmentCount = #distances - 1
    assert(segmentCount > 0, NOT_ENOUGH_POINTS)
    if distance <= 0 then
        return 1, 0
    elseif distance >= distances[segmentCount + 1] then
        return segmentCount, self.segmentLengths[segmentCount]
    end

    local index
    if hint then
        index = hint
        if index < 1 or index > segmentCount or index % 1 ~= 0 then
            index = math.clamp(index // 1, 1, segmentCount)
        end
    else
        local bucket = distance * self.bucketScale // 1 + 1
        if bucket > segmentCount * BUCKETS_PER_SEGMENT then
            bucket = segmentCount * BUCKETS_PER_SEGMENT
        end
        index = self.buckets[bucket]
    end
    while distances[index + 1] <= distance do
        index += 1
    end
    while distances[index] > distance do
        index -= 1
    end
    return index, distance - distances[index]
end

-- Segment `index`'s direction. A segment with no length borrows the nearest direction, looking ahead first, so a
-- spline that starts or ends with a repeated point still has a direction there.
local function getDirection(self: LinearSplineData, index: number): vector
    local directions = self.directions
    local direction = directions[index]
    if direction ~= vector.zero then
        return direction
    end
    for after = index + 1, #directions do
        if directions[after] ~= vector.zero then
            return directions[after]
        end
    end
    for before = index - 1, 1, -1 do
        if directions[before] ~= vector.zero then
            return directions[before]
        end
    end
    return vector.zero
end

function LinearSpline.new(points: { VectorLike }, pivot: VectorLike?): LinearSpline
    local data: LinearSplineData = {
        pivot = if pivot then Vector.from(pivot) else vector.zero,
        points = Vector.fromList(points),
        directions = {},
        segmentSides = {},
        sides = {},
        segmentLengths = {},
        distances = false,
        cachedLength = 0,
        buckets = {},
        bucketScale = 0,
    }
    local self = setmetatable(data, LinearSpline)
    rebuild(self)
    return self
end

--[=[
    @within LinearSpline
    @method getPos
    @param alpha number -- From 0 (the start) to 1 (the end). Clamped.
    @return vector
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    The position at `alpha`. Alpha splits evenly across segments, so with 4 segments, 0.25 is the end of the first.
    Since segments have different lengths, equal alpha steps don't move at a constant speed; use
    [LinearSpline:getUniformPos] or [LinearSpline:getPosAtDistance] for that.
]=]
function LinearSpline.getPos(self: LinearSplineData, alpha: number): vector
    local points = self.points
    local segmentCount = #points - 1
    assert(segmentCount > 0, NOT_ENOUGH_POINTS)
    local u = math.clamp(alpha, 0, 1) * segmentCount
    local index = math.min(math.floor(u) + 1, segmentCount)
    local start: vector = points[index]
    local finish: vector = points[index + 1]
    return self.pivot + start + (finish - start) * (u - (index - 1))
end

--[=[
    @within LinearSpline
    @method sample
    @param count number
    @param out {vector}? -- A list to fill instead of making a new one. Anything past `count` is removed.
    @return {vector} -- `out`, if given.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    `count` positions at evenly spaced alphas, including both ends. Faster than calling [LinearSpline:getPos] in a
    loop. Use [LinearSpline:sampleUniform] for positions evenly spaced along the length instead.
]=]
function LinearSpline.sample(self: LinearSplineData, count: number, out: { vector }?): { vector }
    local result = Vector.output(out, count)
    if count <= 1 then
        if count == 1 then
            result[1] = LinearSpline.getPos(self, 0)
        end
        return result
    end
    local points = self.points
    local segmentCount = #points - 1
    assert(segmentCount > 0, NOT_ENOUGH_POINTS)
    local pivot = self.pivot
    local step = segmentCount / (count - 1)
    for i = 0, count - 1 do
        local u = i * step
        local index = math.min(math.floor(u) + 1, segmentCount)
        local start: vector = points[index]
        local finish: vector = points[index + 1]
        result[i + 1] = pivot + start + (finish - start) * (u - (index - 1))
    end
    return result
end

--[=[
    @within LinearSpline
    @method getLength
    @return number

    The length along the spline, or 0 if it has no segments.
]=]
function LinearSpline.getLength(self: LinearSplineData): number
    if not self.distances then
        getDistances(self)
    end
    return self.cachedLength
end

--[=[
    @within LinearSpline
    @method getAlphaAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return number -- The alpha, as used by [LinearSpline:getPos].
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    The alpha at which `distance` along the spline has been travelled.
]=]
function LinearSpline.getAlphaAtDistance(self: LinearSplineData, distance: number, hint: number?): (number, number)
    local index, offset = locate(self, distance, hint)
    local segmentLength = self.segmentLengths[index]
    local t = if segmentLength > 0 then offset / segmentLength else 0
    return (index - 1 + t) / (#self.points - 1), index
end

--[=[
    @within LinearSpline
    @method getPosAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

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
function LinearSpline.getPosAtDistance(self: LinearSplineData, distance: number, hint: number?): (vector, number)
    local distances = self.distances or getDistances(self)
    local segmentCount = #distances - 1
    local points = self.points
    if distance <= 0 then
        assert(segmentCount > 0, NOT_ENOUGH_POINTS)
        return self.pivot + points[1], 1
    elseif distance >= distances[segmentCount + 1] then
        assert(segmentCount > 0, NOT_ENOUGH_POINTS)
        return self.pivot + points[segmentCount + 1], segmentCount
    end

    local index
    if hint then
        index = hint
        if index < 1 or index > segmentCount or index % 1 ~= 0 then
            index = math.clamp(index // 1, 1, segmentCount)
        end
    else
        local bucket = distance * self.bucketScale // 1 + 1
        if bucket > segmentCount * BUCKETS_PER_SEGMENT then
            bucket = segmentCount * BUCKETS_PER_SEGMENT
        end
        index = self.buckets[bucket]
    end
    while distances[index + 1] <= distance do
        index += 1
    end
    while distances[index] > distance do
        index -= 1
    end

    local start: vector = points[index]
    local direction: vector = self.directions[index]
    return self.pivot + start + direction * (distance - distances[index]), index
end

--[=[
    @within LinearSpline
    @method getUniformPos
    @param alpha number -- From 0 (the start) to 1 (the end), as a fraction of the length. Clamped.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    Like [LinearSpline:getPos], but alpha is a fraction of the length, so equal alpha steps move at a constant
    speed. The same as [LinearSpline:getPosAtDistance] with `alpha * length`.
]=]
function LinearSpline.getUniformPos(self: LinearSplineData, alpha: number, hint: number?): (vector, number)
    return LinearSpline.getPosAtDistance(self, math.clamp(alpha, 0, 1) * LinearSpline.getLength(self), hint)
end

--[=[
    @within LinearSpline
    @method getDirectionAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector -- A unit vector.
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    The direction of the segment `distance` studs along the spline. It changes all at once at each point. A
    spline with no length at all has a zero direction.
]=]
function LinearSpline.getDirectionAtDistance(self: LinearSplineData, distance: number, hint: number?): (vector, number)
    local index = locate(self, distance, hint)
    return getDirection(self, index), index
end

--[=[
    @within LinearSpline
    @method getPosAndDirectionAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector -- The position.
    @return vector -- The direction of travel, as in [LinearSpline:getDirectionAtDistance].
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    [LinearSpline:getPosAtDistance] and [LinearSpline:getDirectionAtDistance] from one lookup.
]=]
function LinearSpline.getPosAndDirectionAtDistance(
    self: LinearSplineData,
    distance: number,
    hint: number?
): (vector, vector, number)
    -- The same walk as getPosAtDistance, repeated here because it's faster inline
    local distances = self.distances or getDistances(self)
    local segmentCount = #distances - 1
    assert(segmentCount > 0, NOT_ENOUGH_POINTS)
    local index
    if distance <= 0 then
        index = 1
        distance = 0
    elseif distance >= distances[segmentCount + 1] then
        index = segmentCount
        distance = distances[segmentCount + 1]
    else
        if hint then
            index = hint
            if index < 1 or index > segmentCount or index % 1 ~= 0 then
                index = math.clamp(index // 1, 1, segmentCount)
            end
        else
            local bucket = distance * self.bucketScale // 1 + 1
            if bucket > segmentCount * BUCKETS_PER_SEGMENT then
                bucket = segmentCount * BUCKETS_PER_SEGMENT
            end
            index = self.buckets[bucket]
        end
        while distances[index + 1] <= distance do
            index += 1
        end
        while distances[index] > distance do
            index -= 1
        end
    end

    local direction: vector = self.directions[index]
    if direction == vector.zero then
        -- Segments with no length are only landed on at the ends, where the offset along them is 0
        direction = getDirection(self, index)
    end
    local start: vector = self.points[index]
    return self.pivot + start + direction * (distance - distances[index]), direction, index
end

--[=[
    @within LinearSpline
    @method getOffsetPosAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param offset number -- Studs to the right of the direction of travel; negative is to the left.
    @param hint number? -- The hint returned by the previous lookup on this spline, if any.
    @return vector -- The offset position.
    @return vector -- The spline's direction of travel there, as in [LinearSpline:getDirectionAtDistance].
    @return number -- A hint for the next lookup.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    The position `offset` studs to the side of the spline, `distance` studs along it, for things that walk beside
    the path instead of on it. The side is horizontal, so the height stays the spline's. Straight up or down has no
    horizontal side, so there it keeps the side of the nearest part of the spline that isn't vertical: a climb that
    bends up and over in one vertical plane keeps the same side the whole way.

    Offset positions jump at each point, where the side flips from one segment's to the next. [Splynx.rounded]
    makes the same path with rounded corners, which offset positions move around smoothly.
]=]
function LinearSpline.getOffsetPosAtDistance(
    self: LinearSplineData,
    distance: number,
    offset: number,
    hint: number?
): (vector, vector, number)
    -- The same as getPosAndDirectionAtDistance, repeated here because it's faster inline
    local distances = self.distances or getDistances(self)
    local segmentCount = #distances - 1
    assert(segmentCount > 0, NOT_ENOUGH_POINTS)
    local index
    if distance <= 0 then
        index = 1
        distance = 0
    elseif distance >= distances[segmentCount + 1] then
        index = segmentCount
        distance = distances[segmentCount + 1]
    else
        if hint then
            index = hint
            if index < 1 or index > segmentCount or index % 1 ~= 0 then
                index = math.clamp(index // 1, 1, segmentCount)
            end
        else
            local bucket = distance * self.bucketScale // 1 + 1
            if bucket > segmentCount * BUCKETS_PER_SEGMENT then
                bucket = segmentCount * BUCKETS_PER_SEGMENT
            end
            index = self.buckets[bucket]
        end
        while distances[index + 1] <= distance do
            index += 1
        end
        while distances[index] > distance do
            index -= 1
        end
    end

    local direction: vector = self.directions[index]
    if direction == vector.zero then
        -- Segments with no length are only landed on at the ends, where the offset along them is 0
        direction = getDirection(self, index)
    end
    local start: vector = self.points[index]
    local position = self.pivot + start + direction * (distance - distances[index])
    return position + self.sides[index] * offset, direction, index
end

--[=[
    @within LinearSpline
    @method getPosAtDistances
    @param distances {number} -- Studs from the start, one per lookup. Clamped to the length.
    @param outPositions {vector} -- Filled with the position for each distance.
    @param outDirections {vector}? -- Filled with the direction of travel for each distance.
    @param offsets {number}? -- One per distance: studs to the right, as in [LinearSpline:getOffsetPosAtDistance].
    @param hints {number}? -- One per distance, read and then replaced with the hint for the next call.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    Many lookups in one call, written into lists you keep, so nothing new is made. Meant for many things moving
    along the same spline: keep their distances (and offsets) in lists, and pass the same hints list every frame.
]=]
function LinearSpline.getPosAtDistances(
    self: LinearSplineData,
    distances: { number },
    outPositions: { vector },
    outDirections: { vector }?,
    offsets: { number }?,
    hints: { number }?
)
    if distances[1] == nil then
        return
    end
    local splineDistances = self.distances or getDistances(self)
    local segmentCount = #splineDistances - 1
    assert(segmentCount > 0, NOT_ENOUGH_POINTS)
    local length = splineDistances[segmentCount + 1]
    local bucketScale = self.bucketScale
    local buckets = self.buckets
    local points = self.points
    local directions = self.directions
    local sides = self.sides
    local pivot = self.pivot
    for i, distance in distances do
        -- The same walk as getPosAtDistance, repeated here because it's faster inline
        local index
        if distance <= 0 then
            index = 1
            distance = 0
        elseif distance >= length then
            index = segmentCount
            distance = length
        else
            local hint = if hints then hints[i] else nil
            if hint then
                index = hint
                if index < 1 or index > segmentCount or index % 1 ~= 0 then
                    index = math.clamp(index // 1, 1, segmentCount)
                end
            else
                local bucket = distance * bucketScale // 1 + 1
                if bucket > segmentCount * BUCKETS_PER_SEGMENT then
                    bucket = segmentCount * BUCKETS_PER_SEGMENT
                end
                index = buckets[bucket]
            end
            while splineDistances[index + 1] <= distance do
                index += 1
            end
            while splineDistances[index] > distance do
                index -= 1
            end
        end

        local direction: vector = directions[index]
        if direction == vector.zero then
            direction = getDirection(self, index)
        end
        local start: vector = points[index]
        local position = pivot + start + direction * (distance - splineDistances[index])
        if offsets then
            position += sides[index] * offsets[i]
        end
        outPositions[i] = position
        if outDirections then
            outDirections[i] = direction
        end
        if hints then
            hints[i] = index
        end
    end
end

--[=[
    @within LinearSpline
    @method sampleUniform
    @param count number
    @param out {vector}? -- A list to fill instead of making a new one. Anything past `count` is removed.
    @return {vector} -- `out`, if given.
    @error "Spline doesn't have enough points to make a segment" -- If the spline has fewer than two points.

    `count` positions evenly spaced along the length, including both ends. Faster than calling
    [LinearSpline:getPosAtDistance] in a loop.
]=]
function LinearSpline.sampleUniform(self: LinearSplineData, count: number, out: { vector }?): { vector }
    local result = Vector.output(out, count)
    if count <= 1 then
        if count == 1 then
            result[1] = LinearSpline.getPos(self, 0)
        end
        return result
    end
    local step = LinearSpline.getLength(self) / (count - 1)
    local points = self.points
    local directions = self.directions
    local pivot = self.pivot
    local hint = 1
    for i = 0, count - 1 do
        local index, offset = locate(self, i * step, hint)
        hint = index
        local start: vector = points[index]
        local direction: vector = directions[index]
        result[i + 1] = pivot + start + direction * offset
    end
    return result
end

--[=[
    @within LinearSpline
    @method getJoints
    @return {vector}

    The segment ends, which are the points with the pivot applied.
]=]
function LinearSpline.getJoints(self: LinearSplineData): { vector }
    local pivot = self.pivot
    local out = table.create(#self.points)
    for i, point in self.points do
        out[i] = pivot + point
    end
    return out
end

--[=[
    @within LinearSpline
    @method getCurveCount
    @return number

    The number of segments.
]=]
function LinearSpline.getCurveCount(self: LinearSplineData): number
    return math.max(#self.points - 1, 0)
end

--[=[
    @within LinearSpline
    @method getJointDistances
    @return {number}

    How far along the spline each point is.
]=]
function LinearSpline.getJointDistances(self: LinearSplineData): { number }
    if #self.points == 0 then
        return {}
    end
    return table.clone(self.distances or getDistances(self))
end

--[=[
    @within LinearSpline
    @method getPoint
    @param index number
    @return vector -- Relative to the pivot.
]=]
function LinearSpline.getPoint(self: LinearSplineData, index: number): vector
    return self.points[index]
end

--[=[
    @within LinearSpline
    @method getPoints
    @return {vector} -- Relative to the pivot.

    A copy of the points. Change them through [LinearSpline:setPoint] and friends so the spline stays up to date.
]=]
function LinearSpline.getPoints(self: LinearSplineData): { vector }
    return table.clone(self.points)
end

--[=[
    @within LinearSpline
    @method setPoint
    @param index number
    @param position VectorLike -- Relative to the pivot.
    @error "No point at index ..." -- If there's no point at `index`.

    Moves one point. Only the two segments that use it are rebuilt.
]=]
function LinearSpline.setPoint(self: LinearSplineData, index: number, position: VectorLike)
    assert(self.points[index], `No point at index {index}`)
    self.points[index] = Vector.from(position)
    updateSegments(self, index - 1, index)
end

--[=[
    @within LinearSpline
    @method setPoints
    @param points {VectorLike} -- Relative to the pivot.

    Replaces every point.
]=]
function LinearSpline.setPoints(self: LinearSplineData, points: { VectorLike })
    self.points = Vector.fromList(points)
    rebuild(self)
end

--[=[
    @within LinearSpline
    @method addPoints
    @param points {VectorLike} -- Relative to the pivot.

    Adds points to the end.
]=]
function LinearSpline.addPoints(self: LinearSplineData, points: { VectorLike })
    local splinePoints = self.points
    local firstNew = #splinePoints + 1
    for i, position in points do
        splinePoints[firstNew + i - 1] = Vector.from(position)
    end
    updateSegments(self, firstNew - 1, #splinePoints - 1)
end

--[=[
    @within LinearSpline
    @method removePoints
    @param indices {number} -- In any order. Repeated indices are removed once.

    Removes several points at once.
]=]
function LinearSpline.removePoints(self: LinearSplineData, indices: { number })
    local sorted = table.clone(indices)
    table.sort(sorted, function(a, b)
        return a > b
    end)
    local previous
    for _, index in sorted do
        if index ~= previous then
            table.remove(self.points, index)
            previous = index
        end
    end
    rebuild(self)
end

--[=[
    @within LinearSpline
    @method removePoint
    @param index number
]=]
function LinearSpline.removePoint(self: LinearSplineData, index: number)
    LinearSpline.removePoints(self, { index })
end

--[=[
    @within LinearSpline
    @method getPivot
    @return vector
]=]
function LinearSpline.getPivot(self: LinearSplineData): vector
    return self.pivot
end

--[=[
    @within LinearSpline
    @method setPivot
    @param pivot VectorLike

    Moves the whole spline. Points stay the same relative to the pivot, so nothing needs rebuilding.
]=]
function LinearSpline.setPivot(self: LinearSplineData, pivot: VectorLike)
    self.pivot = Vector.from(pivot)
end

return LinearSpline
