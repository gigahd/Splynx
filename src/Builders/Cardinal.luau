--!native
--!optimize 2

-- Cardinal: a Hermite spline whose velocities come from the neighbouring points, (next - previous) * scale.
-- A scale of 0.5 is Catmull-Rom. The end points mirror their only neighbour to get a velocity.

local Hermite = require("./Hermite")

local Cardinal = {}
Cardinal.usesScale = true

function Cardinal.curveCount(pointCount: number): number
    return math.max(pointCount - 1, 0)
end

-- Moving a point changes its neighbours' velocities too, so the curves either side of those change as well
function Cardinal.affectedCurves(pointIndex: number): (number, number)
    return pointIndex - 2, pointIndex + 1
end

function Cardinal.velocity(points: { vector }, index: number, scale: number): vector
    -- A velocity needs at least one neighbour
    if #points < 2 then
        return vector.zero
    end
    local point = points[index]
    local previous = if index > 1 then points[index - 1] else 2 * point - points[index + 1]
    local nextPoint = if index < #points then points[index + 1] else 2 * point - points[index - 1]
    return (nextPoint - previous) * scale
end

function Cardinal.coefficients(
    inputs: { points: { vector }, scale: number },
    index: number
): (vector, vector, vector, vector)
    local points, scale = inputs.points, inputs.scale
    return Hermite.fromVelocities(
        points[index],
        Cardinal.velocity(points, index, scale),
        points[index + 1],
        Cardinal.velocity(points, index + 1, scale)
    )
end

return Cardinal
