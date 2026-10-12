--!native
--!optimize 2

-- Cubic Hermite: curve i runs from point i to point i + 1, leaving each point with that point's velocity

local Hermite = {}
Hermite.usesVelocities = true

function Hermite.curveCount(pointCount: number): number
    return math.max(pointCount - 1, 0)
end

function Hermite.affectedCurves(pointIndex: number): (number, number)
    return pointIndex - 1, pointIndex
end

-- Polynomial coefficients of the curve from p0 to p1 with velocities v0 and v1
function Hermite.fromVelocities(p0: vector, v0: vector, p1: vector, v1: vector): (vector, vector, vector, vector)
    return
        p0,
        v0,
        -3 * p0 - 2 * v0 + 3 * p1 - v1,
        2 * p0 + v0 - 2 * p1 + v1
end

function Hermite.coefficients(points: { vector }, index: number, velocities: { vector }?, _scale: number): (vector, vector, vector, vector)
    local pointVelocities = velocities :: { vector }
    return Hermite.fromVelocities(points[index], pointVelocities[index], points[index + 1], pointVelocities[index + 1])
end

return Hermite
