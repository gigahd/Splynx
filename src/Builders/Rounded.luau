--!native
--!optimize 2

-- Rounded corners: straight lines between the points, with the corner at each point replaced by a circular arc
-- tangent to both lines. Curve 2j - 1 is the line along segment j (points j to j + 1) and curve 2k - 2 is the arc at
-- corner k (point k). The end points have no corner.
--
-- Each point has a size, which is either how far along each line its arc reaches from the corner (`byReach`), or the
-- arc's radius (`byRadius`), which reaches `radius * tan(turn / 2)`: further the sharper the turn. Where the arcs at
-- both ends of a segment would reach past each other, both shrink in proportion until they meet.

local Bezier = require("./Bezier")

type Inputs = { points: { vector }, radii: { number }? }

-- Below this, two segments point opposite ways and the path doubles back. No arc fits, so the corner stays sharp.
local DOUBLE_BACK = -1 + 1e-6

-- Segment j's direction and length
local function segment(points: { vector }, j: number): (vector, number)
    local delta = points[j + 1] - points[j]
    local length = vector.magnitude(delta)
    return if length > 0 then delta / length else vector.zero, length
end

-- How far along each line an arc of `size` turning through `turn` radians reaches from its corner
type ReachOf = (size: number, turn: number) -> number

local function newBuilder(reachOf: ReachOf)
    local Rounded = {}
    Rounded.usesRadii = true

    -- How far along each line corner k's arc would reach if it had room
    local function wantedReach(inputs: Inputs, k: number): number
        local points = inputs.points
        if k <= 1 or k >= #points then
            return 0
        end
        local size = (inputs.radii :: { number })[k]
        if size <= 0 then
            return 0
        end
        local before, beforeLength = segment(points, k - 1)
        local after, afterLength = segment(points, k)
        local cosine = vector.dot(before, after)
        if beforeLength <= 0 or afterLength <= 0 or cosine <= DOUBLE_BACK then
            return 0
        end
        return reachOf(size, math.acos(math.min(cosine, 1)))
    end

    -- The fraction of their reach two arcs at either end of a segment get, so they don't overlap
    local function share(segmentLength: number, wanted: number): number
        return if wanted > segmentLength then segmentLength / wanted else 1
    end

    -- How far along each line corner k's arc actually reaches
    local function reach(inputs: Inputs, k: number): number
        local wanted = wantedReach(inputs, k)
        if wanted <= 0 then
            return 0
        end
        local points = inputs.points
        local _, beforeLength = segment(points, k - 1)
        local _, afterLength = segment(points, k)
        return wanted
            * math.min(
                share(beforeLength, wantedReach(inputs, k - 1) + wanted),
                share(afterLength, wanted + wantedReach(inputs, k + 1))
            )
    end

    function Rounded.curveCount(pointCount: number): number
        return math.max(2 * pointCount - 3, 0)
    end

    -- An arc's reach depends on the points up to two away (through how neighbouring arcs share segments), and a line
    -- depends on the arcs at both of its ends
    function Rounded.affectedCurves(pointIndex: number): (number, number)
        return 2 * pointIndex - 7, 2 * pointIndex + 3
    end

    function Rounded.coefficients(inputs: Inputs, index: number): (vector, vector, vector, vector)
        local points = inputs.points
        if index % 2 == 1 then
            -- The line along segment j, from where the arc before it ends to where the arc after it starts
            local j = (index + 1) // 2
            local direction, length = segment(points, j)
            local startReach = reach(inputs, j)
            local remaining = math.max(length - startReach - reach(inputs, j + 1), 0)
            return points[j] + direction * startReach, direction * remaining, vector.zero, vector.zero
        end

        -- The arc at corner k as a cubic Bezier, with its handles along the lines. For an arc turning through `turn`,
        -- the usual handle length of 4/3 * tan(turn / 4) * radius works out to this in terms of the reach.
        local k = index // 2 + 1
        local before = segment(points, k - 1)
        local after = segment(points, k)
        local cornerReach = reach(inputs, k)
        local corner = points[k]
        local from = corner - before * cornerReach
        local to = corner + after * cornerReach
        local quarterTan = math.tan(math.acos(math.clamp(vector.dot(before, after), -1, 1)) / 4)
        local handle = cornerReach * 2 / 3 * (1 - quarterTan * quarterTan)
        return Bezier.fromPoints(from, from + before * handle, to - after * handle, to)
    end

    return Rounded
end

return {
    -- Each point's size is how far along each line its corner's arc reaches
    byReach = newBuilder(function(size, _turn)
        return size
    end),
    -- Each point's size is its corner's arc radius
    byRadius = newBuilder(function(size, turn)
        return size * math.tan(turn / 2)
    end),
}
