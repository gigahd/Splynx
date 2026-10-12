--!native
--!optimize 2

-- Uniform cubic B-spline: curve j uses points j to j + 3. It doesn't pass through its points, but it's smooth
-- everywhere, including across joints.

local BSpline = {}

function BSpline.curveCount(pointCount: number): number
    return math.max(pointCount - 3, 0)
end

function BSpline.affectedCurves(pointIndex: number): (number, number)
    return pointIndex - 3, pointIndex
end

function BSpline.coefficients(points: { vector }, index: number, _velocities: { vector }?, _scale: number): (vector, vector, vector, vector)
    local p0 = points[index]
    local p1 = points[index + 1]
    local p2 = points[index + 2]
    local p3 = points[index + 3]

    return
        1/6 * (p0 + 4 * p1 + p2),
        1/6 * (-3 * p0 + 3 * p2),
        1/6 * (3 * p0 - 6 * p1  + 3 * p2),
        1/6 * (-p0 + 3 * p1 - 3 * p2 + p3)
end

return BSpline
