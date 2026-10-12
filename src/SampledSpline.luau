--!native
--!optimize 2

local Vector = require("./Vector")

type VectorLike = Vector.VectorLike

type Source = {
    getLength: (self: any) -> number,
    getPosAtDistance: (self: any, distance: number, hint: number?) -> (vector, number),
    getPivot: (self: any) -> vector,
    getJointDistances: (self: any) -> { number },
}

-- Segments shorter than this have no direction of their own
local MIN_SEGMENT_LENGTH = 1e-6

--[=[
    @class SampledSpline

    A static copy of a spline as straight segments between samples along it, made by [Splynx.sampled]. Finding
    the segment for a distance takes a single division and a table read instead of a search, which makes it the
    fastest spline for distance lookups.

    Between samples it's a straight line, so it's off by about `spacing^2 / (8 * radius)` on a bend of that
    radius. Sharp corners get cut across by up to half the spacing, unless it keeps the joints, which puts a
    sample on every corner.

    It only knows distances, so alpha is always a fraction of the length: [SampledSpline:getPos] moves at a
    constant speed.
]=]
local SampledSpline = {}
SampledSpline.__index = SampledSpline

--[=[
    @within SampledSpline
    @interface Segments
    .points {vector} -- The samples, relative to the pivot. There's one more than there are segments.
    .directions {vector} -- Each segment's direction, as a unit vector. Segments with no length borrow a neighbour's.
    .distances {number} -- How far along the spline each sample is.
    .count number -- The number of segments.

    A sampled spline's own data, frozen so it can be shared instead of copied. See [SampledSpline:getSegments].
]=]
export type Segments = {
    read points: { vector },
    read directions: { vector },
    read distances: { number },
    read count: number,
}

type SampledSplineData = {
    pivot: vector,
    spacing: number,
    inverseSpacing: number,
    length: number,
    segmentCount: number,
    points: { vector },
    distances: { number },
    directions: { vector },
    -- Each segment's right, for offsets
    sides: { vector },
    -- Each segment's change in position per stud along the spline
    steps: { vector },
    -- The first segment in each `spacing`-long stretch of the spline, so finding a segment is a division and a
    -- short walk
    cells: { number },
    cellCount: number,
    -- Samples exactly `spacing` apart, so a distance's segment is its cell, with no walk
    evenlySpaced: boolean,
    segments: Segments,
}

export type SampledSpline = setmetatable<SampledSplineData, typeof(SampledSpline)>

-- Where to sample `source`: every `spacing` studs, or with `keepJoints`, every joint plus evenly spaced samples no
-- more than `spacing` apart in between
local function sampleDistances(source: Source, spacing: number, keepJoints: boolean): { number }
    local length = source:getLength()
    if not keepJoints then
        local segmentCount = math.max(math.ceil(length / spacing), 1)
        local distances = table.create(segmentCount + 1)
        for i = 0, segmentCount do
            distances[i + 1] = math.min(i * spacing, length)
        end
        return distances
    end

    local joints = source:getJointDistances()
    local distances = { 0 }
    for i = 1, #joints - 1 do
        local start = joints[i]
        local curveLength = joints[i + 1] - start
        if curveLength > 0 then
            local count = math.ceil(curveLength / spacing)
            for k = 1, count - 1 do
                table.insert(distances, start + curveLength * k / count)
            end
            table.insert(distances, joints[i + 1])
        end
    end
    -- A spline with no length still gets one segment
    if #distances == 1 then
        distances[2] = 0
    end
    return distances
end

function SampledSpline.new(source: Source, spacing: number, keepJoints: boolean?): SampledSpline
    assert(spacing > 0, "Spacing must be more than 0")
    local length = source:getLength()
    local pivot = source:getPivot()
    local distances = sampleDistances(source, spacing, keepJoints == true)
    local segmentCount = #distances - 1

    local points = table.create(segmentCount + 1)
    local hint
    for i, distance in distances do
        local position, newHint = source:getPosAtDistance(distance, hint)
        hint = newHint
        points[i] = position - pivot
    end

    local steps = table.create(segmentCount)
    local directions = table.create(segmentCount)
    for i = 1, segmentCount do
        local delta = points[i + 1] - points[i]
        local span = distances[i + 1] - distances[i]
        steps[i] = if span > 0 then delta / span else vector.zero
        local chord = vector.magnitude(delta)
        -- Segments too short to have a direction borrow a neighbour's
        directions[i] = if chord > MIN_SEGMENT_LENGTH then delta / chord else vector.zero
    end
    Vector.fillGaps(directions)
    -- and vertical ones borrow a neighbour's side
    local sides = table.create(segmentCount)
    for i, direction in directions do
        sides[i] = Vector.rightOf(direction)
    end
    Vector.fillGaps(sides)

    local inverseSpacing = 1 / spacing
    local cellCount = math.max(math.ceil(length * inverseSpacing), 1)
    local cells = table.create(cellCount)
    local index = 1
    for cell = 1, cellCount do
        local start = (cell - 1) * spacing
        while index < segmentCount and distances[index + 1] <= start do
            index += 1
        end
        cells[cell] = index
    end

    table.freeze(points)
    table.freeze(distances)
    table.freeze(directions)
    local data: SampledSplineData = {
        pivot = pivot,
        spacing = spacing,
        inverseSpacing = inverseSpacing,
        length = length,
        segmentCount = segmentCount,
        points = points,
        distances = distances,
        directions = directions,
        sides = sides,
        steps = steps,
        cells = cells,
        cellCount = cellCount,
        evenlySpaced = keepJoints ~= true,
        segments = table.freeze({
            points = points,
            directions = directions,
            distances = distances,
            count = segmentCount,
        }),
    }
    return setmetatable(data, SampledSpline)
end

-- The segment `distance` is on, and the distance clamped to the length. The position lookups and
-- getPosAtDistances repeat this inline because it's faster, so change them together.
local function locate(self: SampledSplineData, distance: number): (number, number)
    local clamped = math.clamp(distance, 0, self.length)
    local segmentCount = self.segmentCount
    local distances = self.distances
    local index = clamped * self.inverseSpacing // 1 + 1
    if self.evenlySpaced then
        if index > segmentCount then
            index = segmentCount
        end
    else
        if index > self.cellCount then
            index = self.cellCount
        end
        index = self.cells[index]
        while index < segmentCount and distances[index + 1] <= clamped do
            index += 1
        end
    end
    return index, clamped
end

--[=[
    @within SampledSpline
    @method getPosAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- Ignored: there's no search to speed up. Accepted so calls match the other splines.
    @return vector
    @return number -- The segment index, in place of a hint.

    The position `distance` studs along the spline.
]=]
function SampledSpline.getPosAtDistance(self: SampledSplineData, distance: number, _hint: number?): (vector, number)
    local clamped = math.clamp(distance, 0, self.length)
    local segmentCount = self.segmentCount
    local distances = self.distances
    local index = clamped * self.inverseSpacing // 1 + 1
    if self.evenlySpaced then
        if index > segmentCount then
            index = segmentCount
        end
    else
        if index > self.cellCount then
            index = self.cellCount
        end
        index = self.cells[index]
        while index < segmentCount and distances[index + 1] <= clamped do
            index += 1
        end
    end
    local start: vector = self.points[index]
    local step: vector = self.steps[index]
    return self.pivot + start + step * (clamped - distances[index]), index
end

--[=[
    @within SampledSpline
    @method getDirectionAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- Ignored: there's no search to speed up. Accepted so calls match the other splines.
    @return vector -- A unit vector.
    @return number -- The segment index, in place of a hint.

    The direction of the segment `distance` studs along the spline. It changes all at once at each sample. A
    spline with no length at all has a zero direction.
]=]
function SampledSpline.getDirectionAtDistance(
    self: SampledSplineData,
    distance: number,
    _hint: number?
): (vector, number)
    local index = locate(self, distance)
    return self.directions[index], index
end

--[=[
    @within SampledSpline
    @method getPosAndDirectionAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- Ignored: there's no search to speed up. Accepted so calls match the other splines.
    @return vector -- The position.
    @return vector -- The direction of travel, as in [SampledSpline:getDirectionAtDistance].
    @return number -- The segment index, in place of a hint.

    [SampledSpline:getPosAtDistance] and [SampledSpline:getDirectionAtDistance] from one lookup.
]=]
function SampledSpline.getPosAndDirectionAtDistance(
    self: SampledSplineData,
    distance: number,
    _hint: number?
): (vector, vector, number)
    local clamped = math.clamp(distance, 0, self.length)
    local segmentCount = self.segmentCount
    local distances = self.distances
    local index = clamped * self.inverseSpacing // 1 + 1
    if self.evenlySpaced then
        if index > segmentCount then
            index = segmentCount
        end
    else
        if index > self.cellCount then
            index = self.cellCount
        end
        index = self.cells[index]
        while index < segmentCount and distances[index + 1] <= clamped do
            index += 1
        end
    end
    local start: vector = self.points[index]
    local step: vector = self.steps[index]
    return self.pivot + start + step * (clamped - distances[index]), self.directions[index], index
end

--[=[
    @within SampledSpline
    @method getOffsetPosAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param offset number -- Studs to the right of the direction of travel; negative is to the left.
    @param hint number? -- Ignored: there's no search to speed up. Accepted so calls match the other splines.
    @return vector -- The offset position.
    @return vector -- The spline's direction of travel there, as in [SampledSpline:getDirectionAtDistance].
    @return number -- The segment index, in place of a hint.

    The position `offset` studs to the side of the spline, `distance` studs along it, for things that walk beside
    the path instead of on it. The side is horizontal, so the height stays the spline's. Straight up or down has no
    horizontal side, so there it keeps the side of the nearest part of the spline that isn't vertical: a climb that
    bends up and over in one vertical plane keeps the same side the whole way.

    Offset positions jump a little at each sample where the direction changes, by about `offset * angle`. Smaller
    spacing makes the jumps smaller.
]=]
function SampledSpline.getOffsetPosAtDistance(
    self: SampledSplineData,
    distance: number,
    offset: number,
    _hint: number?
): (vector, vector, number)
    -- The same as getPosAndDirectionAtDistance, repeated here because it's faster inline
    local clamped = math.clamp(distance, 0, self.length)
    local segmentCount = self.segmentCount
    local distances = self.distances
    local index = clamped * self.inverseSpacing // 1 + 1
    if self.evenlySpaced then
        if index > segmentCount then
            index = segmentCount
        end
    else
        if index > self.cellCount then
            index = self.cellCount
        end
        index = self.cells[index]
        while index < segmentCount and distances[index + 1] <= clamped do
            index += 1
        end
    end
    local start: vector = self.points[index]
    local step: vector = self.steps[index]
    local direction: vector = self.directions[index]
    local position = self.pivot + start + step * (clamped - distances[index])
    return position + self.sides[index] * offset, direction, index
end

--[=[
    @within SampledSpline
    @method getPosAtDistances
    @param distances {number} -- Studs from the start, one per lookup. Clamped to the length.
    @param outPositions {vector} -- Filled with the position for each distance.
    @param outDirections {vector}? -- Filled with the direction of travel for each distance.
    @param offsets {number}? -- One per distance: studs to the right, as in [SampledSpline:getOffsetPosAtDistance].
    @param hints {number}? -- Filled with each lookup's segment index, so calls match the other splines.

    Many lookups in one call, written into lists you keep, so nothing new is made.
]=]
function SampledSpline.getPosAtDistances(
    self: SampledSplineData,
    distances: { number },
    outPositions: { vector },
    outDirections: { vector }?,
    offsets: { number }?,
    hints: { number }?
)
    local length = self.length
    local inverseSpacing = self.inverseSpacing
    local cellCount = self.cellCount
    local cells = self.cells
    local evenlySpaced = self.evenlySpaced
    local splineDistances = self.distances
    local segmentCount = self.segmentCount
    local points = self.points
    local steps = self.steps
    local pivot = self.pivot
    local directions = self.directions
    local sides = self.sides
    for i, distance in distances do
        local clamped = math.clamp(distance, 0, length)
        local index = clamped * inverseSpacing // 1 + 1
        if evenlySpaced then
            if index > segmentCount then
                index = segmentCount
            end
        else
            if index > cellCount then
                index = cellCount
            end
            index = cells[index]
            while index < segmentCount and splineDistances[index + 1] <= clamped do
                index += 1
            end
        end
        local start: vector = points[index]
        local step: vector = steps[index]
        local position = pivot + start + step * (clamped - splineDistances[index])
        if offsets then
            position += sides[index] * offsets[i]
        end
        outPositions[i] = position
        if outDirections then
            outDirections[i] = directions[index]
        end
        if hints then
            hints[i] = index
        end
    end
end

--[=[
    @within SampledSpline
    @method getPos
    @param alpha number -- From 0 (the start) to 1 (the end), as a fraction of the length. Clamped.
    @return vector

    The position at a fraction of the length. Unlike the other splines, this moves at a constant speed.
]=]
function SampledSpline.getPos(self: SampledSplineData, alpha: number): vector
    local clamped = math.clamp(alpha, 0, 1) * self.length
    local segmentCount = self.segmentCount
    local distances = self.distances
    local index = clamped * self.inverseSpacing // 1 + 1
    if self.evenlySpaced then
        if index > segmentCount then
            index = segmentCount
        end
    else
        if index > self.cellCount then
            index = self.cellCount
        end
        index = self.cells[index]
        while index < segmentCount and distances[index + 1] <= clamped do
            index += 1
        end
    end
    local start: vector = self.points[index]
    local step: vector = self.steps[index]
    return self.pivot + start + step * (clamped - distances[index])
end

--[=[
    @within SampledSpline
    @method getUniformPos
    @param alpha number -- From 0 (the start) to 1 (the end), as a fraction of the length. Clamped.
    @param hint number? -- Ignored: there's no search to speed up. Accepted so calls match the other splines.
    @return vector
    @return number -- The segment index, in place of a hint.

    The same as [SampledSpline:getPos]; here so calls match the other splines.
]=]
function SampledSpline.getUniformPos(self: SampledSplineData, alpha: number, _hint: number?): (vector, number)
    return SampledSpline.getPosAtDistance(self, math.clamp(alpha, 0, 1) * self.length)
end

--[=[
    @within SampledSpline
    @method getAlphaAtDistance
    @param distance number -- Studs from the start. Clamped to the length.
    @param hint number? -- Ignored: there's no search to speed up. Accepted so calls match the other splines.
    @return number -- `distance / length`.
    @return number -- The segment index, in place of a hint.
]=]
function SampledSpline.getAlphaAtDistance(self: SampledSplineData, distance: number, _hint: number?): (number, number)
    local length = self.length
    local index, clamped = locate(self, distance)
    return if length > 0 then clamped / length else 0, index
end

--[=[
    @within SampledSpline
    @method sampleUniform
    @param count number
    @param out {vector}? -- A list to fill instead of making a new one. Anything past `count` is removed.
    @return {vector} -- `out`, if given.

    `count` positions evenly spaced along the length, including both ends.
]=]
function SampledSpline.sampleUniform(self: SampledSplineData, count: number, out: { vector }?): { vector }
    local result = Vector.output(out, count)
    if count <= 1 then
        if count == 1 then
            result[1] = SampledSpline.getPos(self, 0)
        end
        return result
    end
    local length = self.length
    local inverseSpacing = self.inverseSpacing
    local cellCount = self.cellCount
    local cells = self.cells
    local evenlySpaced = self.evenlySpaced
    local distances = self.distances
    local segmentCount = self.segmentCount
    local points = self.points
    local steps = self.steps
    local pivot = self.pivot
    local distanceStep = length / (count - 1)
    for i = 0, count - 1 do
        local distance = math.min(i * distanceStep, length)
        local index = distance * inverseSpacing // 1 + 1
        if evenlySpaced then
            if index > segmentCount then
                index = segmentCount
            end
        else
            if index > cellCount then
                index = cellCount
            end
            index = cells[index]
            while index < segmentCount and distances[index + 1] <= distance do
                index += 1
            end
        end
        local start: vector = points[index]
        local step: vector = steps[index]
        result[i + 1] = pivot + start + step * (distance - distances[index])
    end
    return result
end

--[=[
    @within SampledSpline
    @method sample
    @param count number
    @param out {vector}? -- A list to fill instead of making a new one. Anything past `count` is removed.
    @return {vector} -- `out`, if given.

    The same as [SampledSpline:sampleUniform], since alpha is already uniform here.
]=]
function SampledSpline.sample(self: SampledSplineData, count: number, out: { vector }?): { vector }
    return SampledSpline.sampleUniform(self, count, out)
end

--[=[
    @within SampledSpline
    @method getLength
    @return number

    The length of the spline it was sampled from, which the distances refer to.
]=]
function SampledSpline.getLength(self: SampledSplineData): number
    return self.length
end

--[=[
    @within SampledSpline
    @method getSpacing
    @return number

    The most studs between samples. Samples are exactly this far apart unless the spline keeps its joints.
]=]
function SampledSpline.getSpacing(self: SampledSplineData): number
    return self.spacing
end

--[=[
    @within SampledSpline
    @method getCurveCount
    @return number

    The number of segments.
]=]
function SampledSpline.getCurveCount(self: SampledSplineData): number
    return self.segmentCount
end

--[=[
    @within SampledSpline
    @method getPoints
    @return {vector} -- Relative to the pivot.

    A copy of the samples. [SampledSpline:getSegments] gives them without copying.
]=]
function SampledSpline.getPoints(self: SampledSplineData): { vector }
    return table.clone(self.points)
end

--[=[
    @within SampledSpline
    @method getSegments
    @return Segments

    The samples, segment directions and distances the spline is made of, for building your own lookups on top of
    it. They're the spline's own lists, frozen rather than copied, so this costs nothing. Points are relative to
    the pivot.

    ```lua
    local segments = sampled:getSegments()
    local pivot = sampled:getPivot()
    for i = 1, segments.count do
        local from = pivot + segments.points[i]
        local direction = segments.directions[i]
        local length = segments.distances[i + 1] - segments.distances[i]
    end
    ```
]=]
function SampledSpline.getSegments(self: SampledSplineData): Segments
    return self.segments
end

--[=[
    @within SampledSpline
    @method getPivot
    @return vector
]=]
function SampledSpline.getPivot(self: SampledSplineData): vector
    return self.pivot
end

--[=[
    @within SampledSpline
    @method setPivot
    @param pivot VectorLike

    Moves the whole spline. Points stay the same relative to the pivot, so nothing needs rebuilding.
]=]
function SampledSpline.setPivot(self: SampledSplineData, pivot: VectorLike)
    self.pivot = Vector.from(pivot)
end

return SampledSpline
