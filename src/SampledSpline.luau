--!native
--!optimize 2

local Vector = require("./Vector")

type VectorLike = Vector.VectorLike

type Source = {
    getLength: (self: any) -> number,
    getPosAtDistance: (self: any, distance: number, hint: number?) -> (vector, number),
    getPivot: (self: any) -> vector,
}

--[=[
    @class SampledSpline

    A static copy of a spline as straight segments a fixed distance apart along it, made by [Splynx.sampled].
    Finding the segment for a distance is a single division instead of a search, which makes it the fastest
    spline for distance lookups.

    Between samples it's a straight line, so it's off by about `spacing^2 / (8 * radius)` on a bend of that
    radius. Sharp corners get cut across by up to half the spacing.

    It only knows distances, so alpha is always a fraction of the length: [SampledSpline:getPos] moves at a
    constant speed.
]=]
local SampledSpline = {}
SampledSpline.__index = SampledSpline

type SampledSplineData = {
    pivot: vector,
    spacing: number,
    inverseSpacing: number,
    length: number,
    segmentCount: number,
    points: { vector },
    steps: { vector },
}

export type SampledSpline = setmetatable<SampledSplineData, typeof(SampledSpline)>

function SampledSpline.new(source: Source, spacing: number): SampledSpline
    assert(spacing > 0, "Spacing must be more than 0")
    local length = source:getLength()
    local pivot = source:getPivot()
    local segmentCount = math.max(math.ceil(length / spacing), 1)

    local points = table.create(segmentCount + 1)
    local hint
    for i = 0, segmentCount do
        local position, newHint = source:getPosAtDistance(math.min(i * spacing, length), hint)
        hint = newHint
        points[i + 1] = position - pivot
    end

    local steps = table.create(segmentCount)
    for i = 1, segmentCount do
        local segmentLength = math.min(i * spacing, length) - (i - 1) * spacing
        steps[i] = if segmentLength > 0 then (points[i + 1] - points[i]) / segmentLength else vector.zero
    end

    local data: SampledSplineData = {
        pivot = pivot,
        spacing = spacing,
        inverseSpacing = 1 / spacing,
        length = length,
        segmentCount = segmentCount,
        points = points,
        steps = steps,
    }
    return setmetatable(data, SampledSpline)
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
    local index = math.min(math.floor(clamped * self.inverseSpacing) + 1, self.segmentCount)
    local start: vector = self.points[index]
    local step: vector = self.steps[index]
    return self.pivot + start + step * (clamped - (index - 1) * self.spacing), index
end

--[=[
    @within SampledSpline
    @method getPos
    @param alpha number -- From 0 (the start) to 1 (the end), as a fraction of the length. Clamped.
    @return vector

    The position at a fraction of the length. Unlike the other splines, this moves at a constant speed.
]=]
function SampledSpline.getPos(self: SampledSplineData, alpha: number): vector
    local distance = math.clamp(alpha, 0, 1) * self.length
    local index = math.min(math.floor(distance * self.inverseSpacing) + 1, self.segmentCount)
    local start: vector = self.points[index]
    local step: vector = self.steps[index]
    return self.pivot + start + step * (distance - (index - 1) * self.spacing)
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
    local clamped = math.clamp(distance, 0, length)
    local index = math.min(math.floor(clamped * self.inverseSpacing) + 1, self.segmentCount)
    return if length > 0 then clamped / length else 0, index
end

--[=[
    @within SampledSpline
    @method sampleUniform
    @param count number
    @return {vector}

    `count` positions evenly spaced along the length, including both ends.
]=]
function SampledSpline.sampleUniform(self: SampledSplineData, count: number): { vector }
    if count <= 1 then
        return if count == 1 then { SampledSpline.getPos(self, 0) } else {}
    end
    local length = self.length
    local spacing = self.spacing
    local inverseSpacing = self.inverseSpacing
    local segmentCount = self.segmentCount
    local points = self.points
    local steps = self.steps
    local pivot = self.pivot
    local distanceStep = length / (count - 1)

    local out = table.create(count)
    for i = 0, count - 1 do
        local distance = math.min(i * distanceStep, length)
        local index = math.min(math.floor(distance * inverseSpacing) + 1, segmentCount)
        local start: vector = points[index]
        local step: vector = steps[index]
        out[i + 1] = pivot + start + step * (distance - (index - 1) * spacing)
    end
    return out
end

--[=[
    @within SampledSpline
    @method sample
    @param count number
    @return {vector}

    The same as [SampledSpline:sampleUniform], since alpha is already uniform here.
]=]
function SampledSpline.sample(self: SampledSplineData, count: number): { vector }
    return SampledSpline.sampleUniform(self, count)
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

    Studs between samples.
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

    A copy of the samples.
]=]
function SampledSpline.getPoints(self: SampledSplineData): { vector }
    return table.clone(self.points)
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
