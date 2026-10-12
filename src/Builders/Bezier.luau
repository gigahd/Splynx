--!native
--!optimize 2

-- Cubic Bezier: curve k uses points 3k - 2 to 3k + 1, so every third point is a joint shared by two curves.
-- Points past the last complete curve are kept but ignored.

local Bezier = {}

function Bezier.curveCount(pointCount: number): number
    return math.max((pointCount - 1) // 3, 0)
end

function Bezier.affectedCurves(pointIndex: number): (number, number)
    return (pointIndex + 1) // 3, (pointIndex + 2) // 3
end

function Bezier.coefficients(points: { vector }, index: number, _velocities: { vector }?, _scale: number): (vector, vector, vector, vector)
    local base = 3 * index - 2
    local p0 = points[base]
    local p1 = points[base + 1]
    local p2 = points[base + 2]
    local p3 = points[base + 3]

    return
        p0,
        -3 * p0 + 3 * p1,
        3 * p0 - 6 * p1 + 3 * p2,
        -p0 + 3 * p1 - 3 * p2 + p3
end

return Bezier
