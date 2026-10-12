-- Host-agnostic correctness checks. Run them with `lune run test`, or in Studio through test.project.json.
-- Returns the number of checks run and the failure messages.

local EPSILON = 1e-4

local v = vector.create

local POINTS = {
    v(0, 0, 0),
    v(10, 12, 0),
    v(20, -6, 4),
    v(30, 4, 0),
    v(40, 14, -3),
    v(50, -4, 2),
    v(60, 6, 0),
}
local VELOCITIES = {
    v(10, 20, 0),
    v(15, 0, 5),
    v(10, -20, 0),
    v(5, 10, 0),
    v(20, 0, -5),
    v(10, -10, 0),
    v(10, 20, 0),
}
local PIVOT = v(100, 5, -50)
-- Big enough that the arcs at both ends of some segments have to share them
local ROUNDED_SIZE = 8

local function expectNear(actual: any, expected: any, label: string, epsilon: number?)
    local tolerance = epsilon or EPSILON
    local difference = if type(actual) == "number" then math.abs(actual - expected) else vector.magnitude(actual - expected)
    if not (difference <= tolerance) then
        error(`{label}: expected {expected}, got {actual}`, 2)
    end
end

local function expect(condition: boolean, label: string)
    if not condition then
        error(label, 2)
    end
end

local function expectError(fn: () -> (), label: string)
    if pcall(fn) then
        error(`{label}: expected an error`, 2)
    end
end

-- Independent reference formulas -----------------------------------------------------------------------------------

type CurveFunction = (t: number) -> vector

local function lerp(a: vector, b: vector, t: number): vector
    return a + (b - a) * t
end

-- de Casteljau's construction
local function bezierPoint(p0: vector, p1: vector, p2: vector, p3: vector, t: number): vector
    local a, b, c = lerp(p0, p1, t), lerp(p1, p2, t), lerp(p2, p3, t)
    return lerp(lerp(a, b, t), lerp(b, c, t), t)
end

-- The Hermite basis functions
local function hermitePoint(p0: vector, v0: vector, p1: vector, v1: vector, t: number): vector
    local t2, t3 = t * t, t * t * t
    return (2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * v0 + (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * v1
end

-- The uniform cubic B-spline weights
local function bSplinePoint(p0: vector, p1: vector, p2: vector, p3: vector, t: number): vector
    local t2, t3 = t * t, t * t * t
    local s = 1 - t
    return (s * s * s * p0 + (3 * t3 - 6 * t2 + 4) * p1 + (-3 * t3 + 3 * t2 + 3 * t + 1) * p2 + t3 * p3) / 6
end

-- Lines between the points with each corner cut by a circular arc, drawn as a cubic Bezier with the usual handle
-- length of 4/3 * tan(turn / 4) * radius. `size` is how far the arc reaches from the corner, or with `byRadius` the
-- arc's radius. Where two arcs would overlap on a segment, both shrink in proportion.
local function roundedCurves(points: { vector }, size: number, byRadius: boolean): { CurveFunction }
    local n = #points
    local function direction(j: number): vector
        return vector.normalize(points[j + 1] - points[j])
    end
    local function turn(k: number): number
        return math.acos(math.clamp(vector.dot(direction(k - 1), direction(k)), -1, 1))
    end
    local function wanted(k: number): number
        if k <= 1 or k >= n then
            return 0
        end
        return if byRadius then size * math.tan(turn(k) / 2) else size
    end
    local function share(j: number): number
        return math.min(1, vector.magnitude(points[j + 1] - points[j]) / (wanted(j) + wanted(j + 1)))
    end
    local function reach(k: number): number
        return if k > 1 and k < n then wanted(k) * math.min(share(k - 1), share(k)) else 0
    end

    local curves = {}
    for j = 1, n - 1 do
        local from = points[j] + direction(j) * reach(j)
        local to = points[j + 1] - direction(j) * reach(j + 1)
        curves[2 * j - 1] = function(t)
            return lerp(from, to, t)
        end
        local k = j + 1
        if k < n then
            local arcRadius = reach(k) / math.tan(turn(k) / 2)
            local handle = 4 / 3 * math.tan(turn(k) / 4) * arcRadius
            local a = points[k] - direction(k - 1) * reach(k)
            local b = points[k] + direction(k) * reach(k)
            local a2, b2 = a + direction(k - 1) * handle, b - direction(k) * handle
            curves[2 * k - 2] = function(t)
                return bezierPoint(a, a2, b2, b, t)
            end
        end
    end
    return curves
end

-- Velocity at a point of a cardinal spline; the end points mirror their neighbour
local function cardinalVelocity(points: { vector }, i: number, scale: number): vector
    local previous = if i > 1 then points[i - 1] else 2 * points[i] - points[i + 1]
    local nextPoint = if i < #points then points[i + 1] else 2 * points[i] - points[i - 1]
    return (nextPoint - previous) * scale
end

type Kind = {
    name: string,
    build: (points: { vector }, velocities: { vector }, pivot: vector?) -> any,
    -- The curves the points should make, relative to the pivot
    reference: (points: { vector }, velocities: { vector }) -> { CurveFunction },
}

local function referencePos(curves: { CurveFunction }, pivot: vector, alpha: number): vector
    local u = math.clamp(alpha, 0, 1) * #curves
    local index = math.min(math.floor(u) + 1, #curves)
    return pivot + curves[index](u - (index - 1))
end

return function(Splynx: any, makeVector3: ((x: number, y: number, z: number) -> any)?): (number, { string })
    local function cardinalKind(name: string, scale: number): Kind
        return {
            name = name,
            build = function(points, _velocities, pivot)
                return Splynx.cardinal(points, scale, pivot)
            end,
            reference = function(points, _velocities)
                local curves = {}
                for i = 1, #points - 1 do
                    local v0, v1 = cardinalVelocity(points, i, scale), cardinalVelocity(points, i + 1, scale)
                    curves[i] = function(t)
                        return hermitePoint(points[i], v0, points[i + 1], v1, t)
                    end
                end
                return curves
            end,
        }
    end

    local KINDS: { Kind } = {
        {
            name = "Bezier",
            build = function(points, _velocities, pivot)
                return Splynx.bezier(points, pivot)
            end,
            reference = function(points, _velocities)
                local curves = {}
                for k = 1, (#points - 1) // 3 do
                    local b = 3 * k - 2
                    curves[k] = function(t)
                        return bezierPoint(points[b], points[b + 1], points[b + 2], points[b + 3], t)
                    end
                end
                return curves
            end,
        },
        {
            name = "Hermite",
            build = function(points, velocities, pivot)
                return Splynx.hermite(points, velocities, pivot)
            end,
            reference = function(points, velocities)
                local curves = {}
                for i = 1, #points - 1 do
                    curves[i] = function(t)
                        return hermitePoint(points[i], velocities[i], points[i + 1], velocities[i + 1], t)
                    end
                end
                return curves
            end,
        },
        cardinalKind("CatmullRom", 0.5),
        cardinalKind("Cardinal", 1),
        {
            name = "Linear",
            build = function(points, _velocities, pivot)
                return Splynx.linear(points, pivot)
            end,
            reference = function(points, _velocities)
                local curves = {}
                for i = 1, #points - 1 do
                    curves[i] = function(t)
                        return lerp(points[i], points[i + 1], t)
                    end
                end
                return curves
            end,
        },
        {
            name = "BSpline",
            build = function(points, _velocities, pivot)
                return Splynx.bSpline(points, pivot)
            end,
            reference = function(points, _velocities)
                local curves = {}
                for j = 1, #points - 3 do
                    curves[j] = function(t)
                        return bSplinePoint(points[j], points[j + 1], points[j + 2], points[j + 3], t)
                    end
                end
                return curves
            end,
        },
    }
    table.insert(KINDS, {
        name = "Rounded",
        build = function(points, _velocities, pivot)
            return Splynx.rounded(points, ROUNDED_SIZE, pivot)
        end,
        reference = function(points, _velocities)
            return roundedCurves(points, ROUNDED_SIZE, false)
        end,
    })
    table.insert(KINDS, {
        name = "Filleted",
        build = function(points, _velocities, pivot)
            return Splynx.filleted(points, ROUNDED_SIZE, pivot)
        end,
        reference = function(points, _velocities)
            return roundedCurves(points, ROUNDED_SIZE, true)
        end,
    })
    -- Catmull-Rom has its own constructor too
    table.insert(KINDS, {
        name = "catmullRom()",
        build = function(points, _velocities, pivot)
            return Splynx.catmullRom(points, pivot)
        end,
        reference = KINDS[3].reference,
    })

    -- A dense polyline along the reference curves, built curve by curve so sharp joints are vertices
    local function referenceWalk(curves: { CurveFunction }, pivot: vector): ({ vector }, { number })
        local positions, distances = { pivot + curves[1](0) }, { 0 }
        for _, curve in curves do
            for i = 1, 2000 do
                local position = pivot + curve(i / 2000)
                table.insert(distances, distances[#distances] + vector.magnitude(position - positions[#positions]))
                table.insert(positions, position)
            end
        end
        return positions, distances
    end

    -- The point `distance` along a reference walk
    local function walkTo(positions: { vector }, distances: { number }, distance: number, from: number): (vector, number)
        local segment = from
        while segment < #positions - 1 and distances[segment + 1] < distance do
            segment += 1
        end
        local span = distances[segment + 1] - distances[segment]
        local fraction = if span > 0 then (distance - distances[segment]) / span else 0
        return positions[segment] + (positions[segment + 1] - positions[segment]) * fraction, segment
    end

    local function expectSameSpline(actual: any, expected: any, label: string)
        expect(actual:getCurveCount() == expected:getCurveCount(), `{label}: curve count`)
        expectNear(actual:getLength(), expected:getLength(), `{label} length`, 1e-3)
        local actualSamples, expectedSamples = actual:sample(64), expected:sample(64)
        for i, position in actualSamples do
            expectNear(position, expectedSamples[i], `{label} sample {i}`)
        end
        local length = expected:getLength()
        for i = 0, 20 do
            expectNear(actual:getPosAtDistance(length * i / 20), expected:getPosAtDistance(length * i / 20), `{label} at distance {i}/20`, 1e-3)
        end
    end

    local checks: { { name: string, run: () -> () } } = {
        {
            name = "getPos matches independent formulas for every spline type",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local curves = kind.reference(POINTS, VELOCITIES)
                    expect(spline:getCurveCount() == #curves, `{kind.name}: expected {#curves} curves`)
                    for i = 0, 97 do
                        expectNear(spline:getPos(i / 97), referencePos(curves, PIVOT, i / 97), `{kind.name} alpha {i}/97`)
                    end
                    for i = 1, #curves + 1 do
                        local alpha = (i - 1) / #curves
                        expectNear(spline:getPos(alpha), referencePos(curves, PIVOT, alpha), `{kind.name} joint {i}`)
                    end
                    expectNear(spline:getPos(-0.5), spline:getPos(0), `{kind.name} alpha below 0`)
                    expectNear(spline:getPos(1.5), spline:getPos(1), `{kind.name} alpha above 1`)
                    expect(type(spline:getPos(0.5)) == "vector", `{kind.name}: getPos didn't return a vector`)
                end
            end,
        },
        {
            name = "getJoints are where the curves meet",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local curves = kind.reference(POINTS, VELOCITIES)
                    local joints = spline:getJoints()
                    expect(#joints == #curves + 1, `{kind.name}: expected {#curves + 1} joints`)
                    for i, curve in curves do
                        expectNear(joints[i], PIVOT + curve(0), `{kind.name} joint {i}`)
                    end
                    expectNear(joints[#joints], PIVOT + curves[#curves](1), `{kind.name} last joint`)
                end
                local bezierJoints = Splynx.bezier(POINTS, PIVOT):getJoints()
                for i, pointIndex in { 1, 4, 7 } do
                    expectNear(bezierJoints[i], PIVOT + POINTS[pointIndex], `Bezier joint {i} is point {pointIndex}`)
                end
            end,
        },
        {
            name = "sample matches getPos at evenly spaced alphas",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local samples = spline:sample(37)
                    expect(#samples == 37, `{kind.name}: wrong sample count`)
                    for i, position in samples do
                        expectNear(position, spline:getPos((i - 1) / 36), `{kind.name} sample {i}`)
                    end
                    expect(#spline:sample(0) == 0, `{kind.name}: sample(0) should be empty`)
                    expectNear(spline:sample(1)[1], spline:getPos(0), `{kind.name} sample(1)`)
                end
            end,
        },
        {
            name = "getPosAtDistance and getLength match a brute-force walk",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local positions, distances = referenceWalk(kind.reference(POINTS, VELOCITIES), PIVOT)
                    local total = distances[#distances]
                    expectNear(spline:getLength(), total, `{kind.name} length`, total * 1e-4)

                    local segment = 1
                    for i = 0, 100 do
                        local distance = total * i / 100
                        local expected
                        expected, segment = walkTo(positions, distances, distance, segment)
                        expectNear(spline:getPosAtDistance(distance), expected, `{kind.name} at distance {distance}`, 0.002)
                    end
                    expectNear(spline:getPosAtDistance(-5), spline:getPos(0), `{kind.name} negative distance`)
                    expectNear(spline:getPosAtDistance(total + 5), spline:getPos(1), `{kind.name} past the end`)
                end
            end,
        },
        {
            name = "straight lines have exact lengths",
            run = function()
                local straight = { v(0, 0, 0), v(10, 0, 0), v(20, 0, 0), v(30, 0, 0) }
                expectNear(Splynx.bezier(straight):getLength(), 30, "Bezier")
                expectNear(Splynx.linear(straight):getLength(), 30, "Linear")
                expectNear(Splynx.catmullRom(straight):getLength(), 30, "CatmullRom")
                expectNear(Splynx.rounded(straight, 5):getLength(), 30, "Rounded")
            end,
        },
        {
            name = "distance lookups agree with each other",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local length = spline:getLength()
                    local previousAlpha = -1
                    for i = 0, 50 do
                        local distance = length * i / 50
                        local alpha = spline:getAlphaAtDistance(distance)
                        expect(alpha >= previousAlpha, `{kind.name}: getAlphaAtDistance isn't increasing at {distance}`)
                        previousAlpha = alpha
                        expectNear(spline:getPos(alpha), spline:getPosAtDistance(distance), `{kind.name} alpha at {distance}`, 1e-3)
                        expectNear(spline:getUniformPos(i / 50), spline:getPosAtDistance(distance), `{kind.name} uniform {i}`)
                    end
                    local uniform = spline:sampleUniform(41)
                    expect(#uniform == 41, `{kind.name}: wrong sampleUniform count`)
                    for i, position in uniform do
                        expectNear(position, spline:getPosAtDistance(length * (i - 1) / 40), `{kind.name} sampleUniform {i}`)
                    end
                end
            end,
        },
        {
            name = "distance hints never change the result",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local length = spline:getLength()
                    local hint
                    -- Forwards and backwards, carrying the hint like a moving object would
                    for i = -5, 405 do
                        local distance = length * i / 400
                        local position, newHint = spline:getPosAtDistance(distance, hint)
                        expectNear(position, spline:getPosAtDistance(distance), `{kind.name} forward {i}`)
                        hint = newHint
                    end
                    for i = 400, 0, -3 do
                        local distance = length * i / 400
                        local alpha, newHint = spline:getAlphaAtDistance(distance, hint)
                        expectNear(alpha, (spline:getAlphaAtDistance(distance)), `{kind.name} backward {i}`)
                        hint = newHint
                    end
                    -- Random jumps with whatever hint the last one left, then nonsense hints
                    for i = 1, 200 do
                        local distance = length * ((i * 0.6180339887) % 1)
                        local position, newHint = spline:getPosAtDistance(distance, hint)
                        expectNear(position, spline:getPosAtDistance(distance), `{kind.name} jump {i}`)
                        hint = newHint
                    end
                    for _, badHint in { 0, -7, 1.5, 1e9, 33, 64 } do
                        expectNear(spline:getPosAtDistance(length * 0.37, badHint), spline:getPosAtDistance(length * 0.37), `{kind.name} hint {badHint}`)
                    end
                    local position, newHint = spline:getUniformPos(0.42, hint)
                    expectNear(position, spline:getUniformPos(0.42), `{kind.name} getUniformPos hint`)
                    expect(type(newHint) == "number", `{kind.name}: getUniformPos didn't return a hint`)

                    -- A hint from before an edit that removed curves
                    local _, staleHint = spline:getPosAtDistance(length * 0.99)
                    spline:removePoints({ 5, 6, 7 })
                    if spline:getCurveCount() > 0 then
                        local shorter = spline:getLength()
                        expectNear(spline:getPosAtDistance(shorter * 0.5, staleHint), spline:getPosAtDistance(shorter * 0.5), `{kind.name} stale hint`)
                    end
                end
            end,
        },
        {
            name = "setPoint matches rebuilding from scratch",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local points = table.clone(POINTS)
                    for index = 1, #points do
                        -- Warm every cache first so a stale one would show up
                        spline:getLength()
                        points[index] += v(index, -2 * index, 3)
                        spline:setPoint(index, points[index])
                        expectNear(spline:getPoint(index), points[index], `{kind.name} getPoint {index}`)
                        expectSameSpline(spline, kind.build(points, VELOCITIES, PIVOT), `{kind.name} after setPoint {index}`)
                    end
                end
            end,
        },
        {
            name = "Hermite setVelocity and Cardinal setScale match rebuilding from scratch",
            run = function()
                local hermite = Splynx.hermite(POINTS, VELOCITIES, PIVOT)
                local velocities = table.clone(VELOCITIES)
                for index = 1, #velocities do
                    hermite:getLength()
                    velocities[index] = v(-index, 3, index * 2)
                    hermite:setVelocity(index, velocities[index])
                    expectNear(hermite:getVelocity(index), velocities[index], `getVelocity {index}`)
                    expectSameSpline(hermite, Splynx.hermite(POINTS, velocities, PIVOT), `Hermite after setVelocity {index}`)
                end

                local cardinal = Splynx.cardinal(POINTS, 0.5, PIVOT)
                cardinal:getLength()
                cardinal:setScale(1.5)
                expect(cardinal:getScale() == 1.5, "getScale")
                expectSameSpline(cardinal, Splynx.cardinal(POINTS, 1.5, PIVOT), "Cardinal after setScale")
                for index = 1, #POINTS do
                    expectNear(cardinal:getVelocity(index), cardinalVelocity(POINTS, index, 1.5), `Cardinal getVelocity {index}`)
                end

                expectError(function()
                    Splynx.bezier(POINTS):setVelocity(1, v(1, 0, 0))
                end, "setVelocity on a Bezier spline")
                expectError(function()
                    Splynx.cardinal(POINTS, 0.5):setVelocity(1, v(1, 0, 0))
                end, "setVelocity on a Cardinal spline")
                expectError(function()
                    Splynx.hermite(POINTS, VELOCITIES):setScale(2)
                end, "setScale on a Hermite spline")
                expectError(function()
                    Splynx.bSpline(POINTS):getVelocity(1)
                end, "getVelocity on a B-spline")
            end,
        },
        {
            name = "addPoints matches rebuilding from scratch",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build({ POINTS[1], POINTS[2] }, { VELOCITIES[1], VELOCITIES[2] }, PIVOT)
                    local count = 2
                    for _, chunk in { 1, 2, 1, 1 } do
                        spline:getLength()
                        local newPoints, newVelocities = {}, {}
                        for i = count + 1, count + chunk do
                            table.insert(newPoints, POINTS[i])
                            table.insert(newVelocities, VELOCITIES[i])
                        end
                        spline:addPoints(newPoints, newVelocities)
                        count += chunk
                        local expected = kind.build(table.move(POINTS, 1, count, 1, {}), table.move(VELOCITIES, 1, count, 1, {}), PIVOT)
                        if expected:getCurveCount() > 0 then
                            expectSameSpline(spline, expected, `{kind.name} after adding up to point {count}`)
                        else
                            expect(spline:getCurveCount() == 0, `{kind.name}: curves from too few points`)
                        end
                    end
                end
                -- Hermite points added without velocities get zero velocity
                local hermite = Splynx.hermite({ POINTS[1], POINTS[2] }, { VELOCITIES[1], VELOCITIES[2] })
                hermite:addPoints({ POINTS[3] })
                expectNear(hermite:getVelocity(3), v(0, 0, 0), "missing Hermite velocity")
            end,
        },
        {
            name = "removePoints removes the given indices in any order",
            run = function()
                for _, build in { Splynx.linear, Splynx.catmullRom } do
                    local function remaining(indices: { number }): string
                        local spline = build({ v(1, 0, 0), v(2, 0, 0), v(3, 0, 0), v(4, 0, 0), v(5, 0, 0) })
                        spline:removePoints(indices)
                        local xs = {}
                        for _, position in spline:getPoints() do
                            table.insert(xs, position.x)
                        end
                        return table.concat(xs, ",")
                    end
                    expect(remaining({ 1, 3 }) == "2,4,5", `removePoints(1, 3) left {remaining({ 1, 3 })}`)
                    expect(remaining({ 3, 1 }) == "2,4,5", `removePoints(3, 1) left {remaining({ 3, 1 })}`)
                    expect(remaining({ 2, 2 }) == "1,3,4,5", `removePoints(2, 2) left {remaining({ 2, 2 })}`)
                    expect(remaining({}) == "1,2,3,4,5", "removePoints({}) changed the spline")
                end
                local hermite = Splynx.hermite(POINTS, VELOCITIES, PIVOT)
                hermite:removePoint(2)
                local points, velocities = table.clone(POINTS), table.clone(VELOCITIES)
                table.remove(points, 2)
                table.remove(velocities, 2)
                expectSameSpline(hermite, Splynx.hermite(points, velocities, PIVOT), "Hermite removePoint keeps velocities in step")
            end,
        },
        {
            name = "the pivot moves the spline without changing its shape",
            run = function()
                for _, kind in KINDS do
                    local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                    local length = spline:getLength()
                    local before = spline:getPosAtDistance(length * 0.3)
                    spline:setPivot(PIVOT + v(0, 0, 10))
                    expectNear(spline:getPivot(), PIVOT + v(0, 0, 10), `{kind.name} getPivot`)
                    expectNear(spline:getPosAtDistance(length * 0.3), before + v(0, 0, 10), `{kind.name} moved`)
                    expectNear(spline:getLength(), length, `{kind.name} length`)
                    expectNear(spline:getPoints()[1], POINTS[1], `{kind.name} points stay relative to the pivot`)
                end
            end,
        },
        {
            name = "too few points make no curves but don't error until queried",
            run = function()
                for _, kind in KINDS do
                    for count = 0, 3 do
                        local spline = kind.build(table.move(POINTS, 1, count, 1, {}), table.move(VELOCITIES, 1, count, 1, {}))
                        if spline:getCurveCount() == 0 then
                            expect(spline:getLength() == 0, `{kind.name} with {count} points: length should be 0`)
                            expect(#spline:sample(0) == 0, `{kind.name} with {count} points: sample(0)`)
                            expectError(function()
                                spline:getPos(0.5)
                            end, `{kind.name} with {count} points: getPos`)
                            for _, distance in { -1, 0, 1 } do
                                for _, method in { "getPosAtDistance", "getDirectionAtDistance", "getPosAndDirectionAtDistance" } do
                                    local ok, err = pcall(spline[method], spline, distance)
                                    expect(not ok and string.find(tostring(err), "enough points") ~= nil, `{kind.name} with {count} points: {method}({distance}) gave {err}`)
                                end
                                local ok, err = pcall(spline.getOffsetPosAtDistance, spline, distance, 1)
                                expect(not ok and string.find(tostring(err), "enough points") ~= nil, `{kind.name} with {count} points: getOffsetPosAtDistance({distance}) gave {err}`)
                                ok, err = pcall(spline.getPosAtDistances, spline, { distance }, {})
                                expect(not ok and string.find(tostring(err), "enough points") ~= nil, `{kind.name} with {count} points: getPosAtDistances({distance}) gave {err}`)
                            end
                            -- Nothing to look up is fine, and empty samples still fill the given list
                            local positions = { v(1, 1, 1) }
                            spline:getPosAtDistances({}, positions)
                            expect(#positions == 1, `{kind.name} with {count} points: getPosAtDistances with nothing to look up wrote something`)
                            expect(spline:sampleUniform(0, positions) == positions and #positions == 0, `{kind.name} with {count} points: sampleUniform(0, out)`)
                        end
                    end
                end
            end,
        },
        {
            name = "sampled splines follow their source",
            run = function()
                for _, kind in KINDS do
                    local source = kind.build(POINTS, VELOCITIES, PIVOT)
                    local length = source:getLength()
                    local coarse = Splynx.sampled(source, 0.5)
                    local fine = Splynx.sampled(source, 0.1)
                    expectNear(coarse:getLength(), length, `{kind.name} sampled length`)
                    expect(coarse:getSpacing() == 0.5, `{kind.name}: getSpacing`)
                    expect(coarse:getCurveCount() == math.ceil(length / 0.5), `{kind.name}: segment count`)

                    -- Exact at the samples, close in between, and closer with smaller spacing
                    for i = 0, math.floor(length / 0.5) do
                        expectNear(coarse:getPosAtDistance(i * 0.5), source:getPosAtDistance(i * 0.5), `{kind.name} sample {i}`, 1e-3)
                    end
                    local coarseError, fineError = 0, 0
                    for i = 0, 1000 do
                        local distance = length * i / 1000
                        local expected = source:getPosAtDistance(distance)
                        coarseError = math.max(coarseError, vector.magnitude(coarse:getPosAtDistance(distance) - expected))
                        fineError = math.max(fineError, vector.magnitude(fine:getPosAtDistance(distance) - expected))
                    end
                    -- Smooth curves stay close; samples either side of a sharp corner (Linear) cut across it, by at
                    -- most half the spacing
                    local tolerance = if kind.name == "Linear" then 0.25 else 0.05
                    expect(coarseError < tolerance, `{kind.name}: spacing 0.5 is off by {coarseError}`)
                    expect(fineError < coarseError, `{kind.name}: spacing 0.1 ({fineError}) isn't closer than 0.5 ({coarseError})`)

                    -- Keeping the joints puts a sample on every corner, so even Linear stays close
                    local kept = Splynx.sampled(source, 0.5, true)
                    expectNear(kept:getLength(), length, `{kind.name} kept joints length`)
                    local keptError = 0
                    for i = 0, 1000 do
                        local distance = length * i / 1000
                        keptError = math.max(keptError, vector.magnitude(kept:getPosAtDistance(distance) - source:getPosAtDistance(distance)))
                    end
                    expect(keptError < (if kind.name == "Linear" then 1e-3 else 0.05), `{kind.name}: keeping joints is off by {keptError}`)

                    expectNear(coarse:getPosAtDistance(-5), source:getPos(0), `{kind.name} sampled start`, 1e-3)
                    expectNear(coarse:getPosAtDistance(length + 5), source:getPos(1), `{kind.name} sampled end`, 1e-3)
                    expectNear(coarse:getPos(0.3), coarse:getPosAtDistance(length * 0.3), `{kind.name} sampled getPos is uniform`)
                    expectNear(coarse:getUniformPos(0.3), coarse:getPosAtDistance(length * 0.3), `{kind.name} sampled getUniformPos`)
                    expectNear(coarse:getAlphaAtDistance(length * 0.3), 0.3, `{kind.name} sampled getAlphaAtDistance`)
                    local uniform = coarse:sampleUniform(21)
                    for i, position in coarse:sample(21) do
                        expectNear(position, uniform[i], `{kind.name} sampled sample {i}`)
                        expectNear(position, coarse:getPosAtDistance(length * (i - 1) / 20), `{kind.name} sampled sampleUniform {i}`)
                    end

                    -- It's a snapshot: the pivot moves it, but editing the source doesn't
                    local before = coarse:getPosAtDistance(length * 0.5)
                    source:setPoint(4, v(0, 100, 0))
                    expectNear(coarse:getPosAtDistance(length * 0.5), before, `{kind.name} sampled snapshot`)
                    coarse:setPivot(PIVOT + v(5, 0, 0))
                    expectNear(coarse:getPosAtDistance(length * 0.5), before + v(5, 0, 0), `{kind.name} sampled pivot`)
                end

                -- Spacing longer than the spline, and a spline with no length
                expect(Splynx.sampled(Splynx.linear({ v(0, 0, 0), v(3, 0, 0) }), 10, true):getCurveCount() == 1, "one kept segment when spacing is longer than the spline")
                local keptPoint = Splynx.sampled(Splynx.linear({ v(1, 2, 3), v(1, 2, 3) }), 1, true)
                expectNear(keptPoint:getPosAtDistance(0.5), v(1, 2, 3), "zero-length spline keeping joints")
                expectNear(keptPoint:getDirectionAtDistance(0.5), v(0, 0, 0), "zero-length spline has no direction")
                local short = Splynx.sampled(Splynx.linear({ v(0, 0, 0), v(3, 0, 0) }), 10)
                expect(short:getCurveCount() == 1, "one segment when spacing is longer than the spline")
                expectNear(short:getPosAtDistance(1.5), v(1.5, 0, 0), "long spacing midpoint")
                local point = Splynx.sampled(Splynx.linear({ v(1, 2, 3), v(1, 2, 3) }), 1)
                expectNear(point:getPosAtDistance(0.5), v(1, 2, 3), "zero-length spline")
                expectError(function()
                    Splynx.sampled(Splynx.linear({ v(0, 0, 0), v(3, 0, 0) }), 0)
                end, "spacing of 0")
            end,
        },
    }

    -- The direction of travel measured from positions either side of `distance`, far enough apart that float32
    -- rounding doesn't matter
    local function measuredDirection(spline: any, distance: number): vector
        return vector.normalize(spline:getPosAtDistance(distance + 0.01) - spline:getPosAtDistance(distance - 0.01))
    end

    -- Distances partway along each curve, away from the joints where Linear and Bezier turn sharply
    local function distancesBetweenJoints(jointDistances: { number }): { number }
        local out = {}
        for i = 1, #jointDistances - 1 do
            local start, finish = jointDistances[i], jointDistances[i + 1]
            if finish - start > 0.01 then
                for _, fraction in { 0.13, 0.5, 0.87 } do
                    table.insert(out, start + (finish - start) * fraction)
                end
            end
        end
        return out
    end

    table.insert(checks, {
        name = "getJointDistances are how far along each joint is",
        run = function()
            for _, kind in KINDS do
                local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                local joints, distances = spline:getJoints(), spline:getJointDistances()
                expect(#distances == #joints, `{kind.name}: {#distances} joint distances for {#joints} joints`)
                expect(distances[1] == 0, `{kind.name}: first joint distance`)
                expectNear(distances[#distances], spline:getLength(), `{kind.name} last joint distance`)
                for i, distance in distances do
                    expectNear(spline:getPosAtDistance(distance), joints[i], `{kind.name} joint {i}`, 1e-3)
                end
                expect(#kind.build({}, {}, PIVOT):getJointDistances() == 0, `{kind.name}: joint distances with no points`)
            end
        end,
    })

    table.insert(checks, {
        name = "directions point along the spline",
        run = function()
            local splines = {}
            for _, kind in KINDS do
                table.insert(splines, { name = kind.name, spline = kind.build(POINTS, VELOCITIES, PIVOT) })
            end
            for _, entry in table.clone(splines) do
                if entry.spline.getJointDistances then
                    table.insert(splines, { name = `{entry.name} sampled`, spline = Splynx.sampled(entry.spline, 0.5) })
                end
            end

            for _, entry in splines do
                local spline, name = entry.spline, entry.name
                local distances
                if spline.getJointDistances then
                    distances = distancesBetweenJoints(spline:getJointDistances())
                else
                    -- Mid-segment for a sampled spline, where its direction is constant
                    distances = {}
                    for i = 1, spline:getCurveCount() - 1, 7 do
                        table.insert(distances, (i - 0.5) * spline:getSpacing())
                    end
                end
                local hint
                for _, distance in distances do
                    local direction, newHint = spline:getDirectionAtDistance(distance, hint)
                    hint = newHint
                    expectNear(vector.magnitude(direction), 1, `{name} direction length at {distance}`)
                    expectNear(direction, measuredDirection(spline, distance), `{name} direction at {distance}`, 2e-3)
                    expectNear(direction, (spline:getDirectionAtDistance(distance)), `{name} direction hint at {distance}`)

                    local position, sameDirection, sameHint = spline:getPosAndDirectionAtDistance(distance)
                    expectNear(position, spline:getPosAtDistance(distance), `{name} getPosAndDirectionAtDistance position`)
                    expectNear(sameDirection, direction, `{name} getPosAndDirectionAtDistance direction`)
                    expect(sameHint == select(2, spline:getPosAtDistance(distance)), `{name}: getPosAndDirectionAtDistance hint`)
                end
                -- Clamped past both ends
                expectNear(spline:getDirectionAtDistance(-5), spline:getDirectionAtDistance(0), `{name} direction before the start`)
                local length = spline:getLength()
                expectNear(spline:getDirectionAtDistance(length + 5), spline:getDirectionAtDistance(length), `{name} direction past the end`)
            end

            -- Hermite curves with no velocities stop at every point, and still have a direction there
            local stopping = Splynx.hermite({ v(0, 0, 0), v(10, 0, 0), v(10, 10, 0) })
            expectNear(stopping:getDirectionAtDistance(0), v(1, 0, 0), "direction leaving a stop", 1e-3)
            expectNear(stopping:getDirectionAtDistance(stopping:getLength()), v(0, 1, 0), "direction arriving at a stop", 1e-3)
            expectNear(stopping:getDirectionAtDistance(5), v(1, 0, 0), "direction between stops", 1e-3)
            -- Far from the origin, rounding can leave a little velocity at a stop, pointing any way
            local far = v(5462.33, 249.80, -9185.84)
            local farStopping = Splynx.hermite({ far, far + v(10, 0, 0), far + v(10, 10, 0) })
            expectNear(farStopping:getDirectionAtDistance(0), v(1, 0, 0), "direction leaving a stop far away", 1e-3)
            expectNear(farStopping:getDirectionAtDistance(farStopping:getLength()), v(0, 1, 0), "direction arriving at a stop far away", 1e-3)

            -- Repeated points make curves with no length, which borrow the nearest direction
            local repeated = { v(1, 0, 0), v(1, 0, 0), v(1, 0, 0), v(1, 0, 0), v(1, 0, 5), v(1, 0, 10), v(1, 0, 10), v(1, 0, 10), v(1, 0, 10) }
            for _, spline in { Splynx.linear(repeated), Splynx.bezier(repeated), Splynx.rounded(repeated, 2) } do
                expectNear(spline:getLength(), 10, "length with repeated points", 1e-3)
                expectNear(spline:getDirectionAtDistance(0), v(0, 0, 1), "direction at a repeated start")
                expectNear(spline:getDirectionAtDistance(10), v(0, 0, 1), "direction at a repeated end")
                expectNear(spline:getPosAtDistance(5), v(1, 0, 5), "position with repeated points", 1e-3)
            end
            expectNear(Splynx.linear({ v(1, 2, 3), v(1, 2, 3) }):getDirectionAtDistance(0), v(0, 0, 0), "no length, no direction")
        end,
    })

    table.insert(checks, {
        name = "offset positions are to the side, at the same height",
        run = function()
            -- Heading along +X, the right is +Z
            local straight = Splynx.linear({ v(0, 0, 0), v(10, 0, 0) }, PIVOT)
            local position, direction, hint = straight:getOffsetPosAtDistance(4, 2)
            expectNear(position, PIVOT + v(4, 0, 2), "right of +X")
            expectNear(direction, v(1, 0, 0), "offset direction")
            expect(hint == 1, "offset hint")
            expectNear((straight:getOffsetPosAtDistance(4, -2)), PIVOT + v(4, 0, -2), "left of +X")

            for _, kind in KINDS do
                local spline = kind.build(POINTS, VELOCITIES, PIVOT)
                local splines = { spline, Splynx.sampled(spline, 0.5) }
                for _, distance in distancesBetweenJoints(spline:getJointDistances()) do
                    for _, each in splines do
                        local offsetPosition, pathDirection = each:getOffsetPosAtDistance(distance, 1.5)
                        local onPath, sameDirection = each:getPosAndDirectionAtDistance(distance)
                        local side = offsetPosition - onPath
                        expectNear(pathDirection, sameDirection, `{kind.name} offset direction`)
                        expectNear(side.y, 0, `{kind.name} offset keeps the height`)
                        expectNear(vector.magnitude(side), 1.5, `{kind.name} offset distance`)
                        expectNear(vector.dot(side, pathDirection), 0, `{kind.name} offset is to the side`)
                        expect(vector.cross(pathDirection, side).y < 0, `{kind.name}: positive offsets are to the right`)
                    end
                end
            end

            -- Straight up keeps the side from the rest of the path. These head along +X, climb straight up a wall
            -- and carry on along +X (or start by climbing), so the right is +Z the whole way.
            for _, path in { { v(0, 0, 0), v(10, 0, 0), v(10, 10, 0), v(20, 10, 0) }, { v(0, 0, 0), v(0, 10, 0), v(10, 10, 0) } } do
                -- A Bezier curve along each segment that stops at both ends
                local bezierPoints = { path[1] }
                for k = 2, #path do
                    table.insert(bezierPoints, path[k - 1])
                    table.insert(bezierPoints, path[k])
                    table.insert(bezierPoints, path[k])
                end
                local splines = { Splynx.linear(path), Splynx.rounded(path, 2), Splynx.filleted(path, 2), Splynx.bezier(bezierPoints) }
                for _, spline in table.clone(splines) do
                    table.insert(splines, Splynx.sampled(spline, 0.5))
                    table.insert(splines, Splynx.sampled(spline, 0.5, true))
                end
                for s, spline in splines do
                    local length = spline:getLength()
                    local distances, offsets, positions = {}, {}, {}
                    for i = 0, 100 do
                        local distance = length * i / 100
                        local offsetPosition = spline:getOffsetPosAtDistance(distance, 1.5)
                        expectNear(offsetPosition - spline:getPosAtDistance(distance), v(0, 0, 1.5), `climbing spline {s} at {distance}`, 1e-4)
                        distances[i + 1], offsets[i + 1] = distance, 1.5
                    end
                    spline:getPosAtDistances(distances, positions, nil, offsets)
                    for i, distance in distances do
                        expectNear(positions[i] - spline:getPosAtDistance(distance), v(0, 0, 1.5), `climbing spline {s} batch at {distance}`, 1e-4)
                    end
                end
            end

            -- A spline that only goes straight up has no side at all
            local vertical = Splynx.linear({ v(0, 0, 0), v(0, 10, 0) })
            expectNear((vertical:getOffsetPosAtDistance(5, 3)), v(0, 5, 0), "vertical offset is ignored")
            expectNear((Splynx.rounded({ v(0, 0, 0), v(0, 10, 0), v(0, 20, 0) }, 2):getOffsetPosAtDistance(5, 3)), v(0, 5, 0), "vertical rounded offset is ignored")
        end,
    })

    table.insert(checks, {
        name = "getPosAtDistances matches single lookups",
        run = function()
            for _, kind in KINDS do
                local source = kind.build(POINTS, VELOCITIES, PIVOT)
                for _, spline in { source, Splynx.sampled(source, 0.5), Splynx.sampled(source, 0.5, true) } do
                    local length = spline:getLength()
                    local distances, offsets = {}, {}
                    for i = 1, 40 do
                        distances[i] = length * ((i * 0.6180339887) % 1.2 - 0.1)
                        offsets[i] = (i % 5) - 2
                    end
                    local positions, directions, hints = {}, {}, {}
                    spline:getPosAtDistances(distances, positions, directions, offsets, hints)
                    expect(#positions == 40 and #directions == 40 and #hints == 40, `{kind.name}: batch output sizes`)
                    for i, distance in distances do
                        local position, direction, hint = spline:getOffsetPosAtDistance(distance, offsets[i])
                        expectNear(positions[i], position, `{kind.name} batch position {i}`)
                        expectNear(directions[i], direction, `{kind.name} batch direction {i}`)
                        expect(hints[i] == hint, `{kind.name}: batch hint {i}`)
                    end

                    -- Moving everything forward a little, reusing the hints and lists
                    for i in distances do
                        distances[i] += 0.37
                    end
                    spline:getPosAtDistances(distances, positions, nil, nil, hints)
                    for i, distance in distances do
                        expectNear(positions[i], spline:getPosAtDistance(distance), `{kind.name} batch with hints {i}`)
                    end
                    spline:getPosAtDistances(distances, positions)
                    for i, distance in distances do
                        expectNear(positions[i], spline:getPosAtDistance(distance), `{kind.name} batch without hints {i}`)
                    end
                end
            end
        end,
    })

    table.insert(checks, {
        name = "sample and sampleUniform fill a given list",
        run = function()
            for _, kind in KINDS do
                local source = kind.build(POINTS, VELOCITIES, PIVOT)
                for _, spline in { source, Splynx.sampled(source, 0.5) } do
                    for _, method in { "sample", "sampleUniform" } do
                        local out = table.create(50, v(9, 9, 9))
                        local result = spline[method](spline, 21, out)
                        expect(result == out, `{kind.name} {method}: didn't return the given list`)
                        expect(#out == 21, `{kind.name} {method}: left {#out} entries`)
                        for i, position in spline[method](spline, 21) do
                            expectNear(out[i], position, `{kind.name} {method} {i}`)
                        end
                        spline[method](spline, 1, out)
                        expect(#out == 1, `{kind.name} {method}(1): left {#out} entries`)
                        spline[method](spline, 0, out)
                        expect(#out == 0, `{kind.name} {method}(0): left {#out} entries`)
                    end
                end
            end
        end,
    })

    table.insert(checks, {
        name = "rounded corners are circular arcs between straight lines",
        run = function()
            -- A right angle with room for the whole radius: lines of 15, then a quarter circle about (15, 0, 5)
            local corner = Splynx.rounded({ v(0, 0, 0), v(20, 0, 0), v(20, 0, 20) }, 5, PIVOT)
            expect(corner:getCurveCount() == 3, "line, arc, line")
            expectNear(corner:getLength(), 30 + math.pi * 2.5, "right angle length", 2e-3)
            local joints = corner:getJoints()
            expectNear(joints[2], PIVOT + v(15, 0, 0), "arc starts")
            expectNear(joints[3], PIVOT + v(20, 0, 5), "arc ends")
            for i = 0, 50 do
                expectNear(corner:getPosAtDistance(15 * i / 50), PIVOT + v(15 * i / 50, 0, 0), `first line {i}`, 1e-3)
                local onArc = corner:getPos((1 + i / 50) / 3)
                expectNear(vector.magnitude(onArc - PIVOT - v(15, 0, 5)), 5, `arc radius {i}`, 2e-3)
            end
            expectNear(corner:getPosAndDirectionAtDistance(15 + math.pi * 1.25), PIVOT + v(15 + 5 / math.sqrt(2), 0, 5 - 5 / math.sqrt(2)), "arc middle", 3e-3)
            expectNear(select(2, corner:getPosAndDirectionAtDistance(15 + math.pi * 1.25)), vector.normalize(v(1, 0, 1)), "arc middle direction", 1e-3)

            -- Corners in any plane
            local upright = Splynx.rounded({ v(0, 0, 0), v(10, 0, 0), v(10, 10, 0) }, 4)
            for i = 0, 20 do
                expectNear(vector.magnitude(upright:getPos((1 + i / 20) / 3) - v(6, 4, 0)), 4, `upright arc {i}`, 2e-3)
            end

            -- A list of radii, with missing ones and 0 leaving the corner sharp like a linear spline
            local zigzag = { v(0, 0, 0), v(10, 0, 0), v(10, 0, 10), v(20, 0, 10), v(20, 0, 20) }
            local sharp = Splynx.rounded(zigzag, { 0, 0, 0 })
            local linear = Splynx.linear(zigzag)
            expectNear(sharp:getLength(), linear:getLength(), "sharp rounded length")
            for i = 0, 20 do
                local distance = linear:getLength() * i / 20
                expectNear(sharp:getPosAtDistance(distance), linear:getPosAtDistance(distance), `sharp rounded {i}`, 1e-3)
            end
            expectSameSpline(Splynx.rounded(zigzag, { 3, 3, 3, 3, 3 }), Splynx.rounded(zigzag, 3), "radii list matches one radius")
            -- Right angles reach as far as their radius, so both kinds agree there
            expectSameSpline(Splynx.filleted(zigzag, 3), Splynx.rounded(zigzag, 3), "filleted right angles")
            local mixed = Splynx.rounded(zigzag, { 0, 2, 0, 3 })
            expect(mixed:getRadius(3) == 0 and mixed:getRadius(4) == 3 and mixed:getRadius(5) == 0, "radii from a list")

            -- Too big a radius shrinks until the arcs meet: two quarter circles of radius 2 and lines of 2
            local tight = Splynx.rounded({ v(0, 0, 0), v(4, 0, 0), v(4, 0, 4), v(8, 0, 4) }, 100)
            expectNear(tight:getLength(), 4 + 2 * math.pi, "shrunk arcs", 3e-3)
            expectNear(tight:getPosAtDistance(2 + math.pi), v(4, 0, 2), "shrunk arcs meet", 1e-3)

            -- Doubling back has no arc that fits, so it turns sharply at the point
            local back = Splynx.rounded({ v(0, 0, 0), v(10, 0, 0), v(0, 0, 0) }, 3)
            expectNear(back:getLength(), 20, "doubling back length")
            expectNear(back:getPosAtDistance(10), v(10, 0, 0), "doubling back reaches the point")
        end,
    })

    table.insert(checks, {
        name = "rounded corners start their reach from the corner at any angle, filleted ones keep their radius",
        run = function()
            -- Heading along +X to a corner at (20, 0, 0), then turning left by `turn`, far enough to have room
            for _, degrees in { 20, 60, 90, 135, 160 } do
                local turn = math.rad(degrees)
                local out = v(math.cos(turn), 0, math.sin(turn))
                local corner = v(20, 0, 0)
                local points = { v(0, 0, 0), corner, corner + out * 500 }
                -- The arc's centre is to the left of the incoming line, `radius` from where the arc starts
                local function expectArc(spline: any, reach: number, radius: number, label: string)
                    local joints = spline:getJoints()
                    expectNear(joints[2], corner - v(reach, 0, 0), `{label} arc starts`, 1e-4)
                    expectNear(joints[3], corner + out * reach, `{label} arc ends`, 1e-4)
                    local centre = corner - v(reach, 0, 0) + v(0, 0, radius)
                    for i = 0, 20 do
                        local onArc = spline:getPos((1 + i / 20) / 3)
                        expectNear(vector.magnitude(onArc - centre), radius, `{label} arc point {i}`, radius * 1e-2)
                    end
                end
                local rounded = Splynx.rounded(points, 2)
                expectArc(rounded, 2, 2 / math.tan(turn / 2), `rounded {degrees} degrees`)
                local filleted = Splynx.filleted(points, 2)
                expectArc(filleted, 2 * math.tan(turn / 2), 2, `filleted {degrees} degrees`)

                -- Moving the far point further away doesn't move where the rounding starts
                rounded:setPoint(3, corner + out * 50)
                expectNear(rounded:getJoints()[2], corner - v(2, 0, 0), `rounded {degrees} degrees after moving a point`, 1e-4)
            end
        end,
    })

    table.insert(checks, {
        name = "rounded and filleted radius edits match rebuilding from scratch",
        run = function()
            for name, build in { rounded = Splynx.rounded, filleted = Splynx.filleted } do
                local radii = { 1, 2, 3, 4, 5, 6, 7 }
                local spline = build(POINTS, radii, PIVOT)
                for index = 1, #POINTS do
                    spline:getLength()
                    radii[index] = 12 - index
                    spline:setRadius(index, radii[index])
                    expect(spline:getRadius(index) == radii[index], `{name} getRadius {index}`)
                    expectSameSpline(spline, build(POINTS, radii, PIVOT), `{name} after setRadius {index}`)
                end
                -- Removing a point keeps the radii in step, and added points get the radius it was made with
                spline:removePoint(3)
                table.remove(radii, 3)
                expectSameSpline(spline, build(table.move(POINTS, 4, 7, 3, { POINTS[1], POINTS[2] }), radii, PIVOT), `{name} after removePoint`)
                local uniform = build({ POINTS[1], POINTS[2] }, 6)
                uniform:addPoints({ POINTS[3], POINTS[4] })
                expect(uniform:getRadius(3) == 6, `{name}: added points get the spline's radius`)
                local listed = build({ POINTS[1], POINTS[2] }, { 6, 6 })
                listed:addPoints({ POINTS[3] })
                expect(listed:getRadius(3) == 0, `{name}: added points get 0 when made with a list`)
                -- setPoints keeps radii in order and fills in new points the same way
                uniform:setPoints({ POINTS[1], POINTS[2], POINTS[3], POINTS[4], POINTS[5] })
                expectSameSpline(uniform, build({ POINTS[1], POINTS[2], POINTS[3], POINTS[4], POINTS[5] }, 6), `{name} after setPoints`)
            end

            expectError(function()
                Splynx.catmullRom(POINTS):setRadius(2, 1)
            end, "setRadius on a Catmull-Rom spline")
            expectError(function()
                Splynx.bezier(POINTS):getRadius(2)
            end, "getRadius on a Bezier spline")
            expectError(function()
                Splynx.rounded(POINTS, 3):getVelocity(2)
            end, "getVelocity on a rounded spline")
            expectError(function()
                Splynx.rounded(POINTS, 3):setRadius(99, 1)
            end, "setRadius past the last point")
        end,
    })

    table.insert(checks, {
        name = "sampled splines keeping their joints",
        run = function()
            for _, kind in KINDS do
                local source = kind.build(POINTS, VELOCITIES, PIVOT)
                local sampled = Splynx.sampled(source, 0.7, true)
                local segments = sampled:getSegments()
                expect(sampled:getSegments() == segments, `{kind.name}: getSegments made a copy`)
                expect(segments.count == sampled:getCurveCount(), `{kind.name}: segment count`)
                expect(#segments.points == segments.count + 1 and #segments.distances == segments.count + 1, `{kind.name}: one more sample than segments`)
                expect(#segments.directions == segments.count, `{kind.name}: one direction per segment`)
                for _, list in { segments, segments.points, segments.directions, segments.distances } do
                    expect(table.isfrozen(list), `{kind.name}: segments aren't frozen`)
                end

                -- Every joint is a sample
                local sampleIndex = 1
                local joints = source:getJoints()
                for i, jointDistance in source:getJointDistances() do
                    while sampleIndex < #segments.distances and segments.distances[sampleIndex] < jointDistance - 1e-6 do
                        sampleIndex += 1
                    end
                    expectNear(segments.distances[sampleIndex], jointDistance, `{kind.name} joint {i} is sampled`, 1e-6)
                    expectNear(sampled:getPivot() + segments.points[sampleIndex], joints[i], `{kind.name} joint {i} position`, 1e-3)
                end

                for i = 1, segments.count do
                    local span = segments.distances[i + 1] - segments.distances[i]
                    expect(span > 0 and span <= 0.7 + 1e-9, `{kind.name}: segment {i} is {span} long`)
                    expectNear(vector.magnitude(segments.directions[i]), 1, `{kind.name} segment {i} direction`)
                    local middle = segments.distances[i] + span / 2
                    expectNear(sampled:getPosAtDistance(middle), sampled:getPivot() + (segments.points[i] + segments.points[i + 1]) / 2, `{kind.name} segment {i} middle`)
                    expectNear(sampled:getDirectionAtDistance(middle), segments.directions[i], `{kind.name} segment {i} lookup direction`)
                end

                -- Every lookup finds the same segments
                local length = sampled:getLength()
                local uniform = sampled:sampleUniform(33)
                for i = 0, 32 do
                    local distance = length * i / 32
                    local position = sampled:getPosAtDistance(distance)
                    local direction = sampled:getDirectionAtDistance(distance)
                    expectNear(uniform[i + 1], position, `{kind.name} kept sampleUniform {i}`)
                    expectNear(sampled:getPos(i / 32), position, `{kind.name} kept getPos {i}`)
                    local samePosition, sameDirection = sampled:getPosAndDirectionAtDistance(distance)
                    expectNear(samePosition, position, `{kind.name} kept getPosAndDirectionAtDistance position {i}`)
                    expectNear(sameDirection, direction, `{kind.name} kept getPosAndDirectionAtDistance direction {i}`)
                    local offsetPosition, offsetDirection = sampled:getOffsetPosAtDistance(distance, 1)
                    expectNear(offsetDirection, direction, `{kind.name} kept getOffsetPosAtDistance direction {i}`)
                    expectNear(vector.magnitude(offsetPosition - position), 1, `{kind.name} kept getOffsetPosAtDistance {i}`)
                end
            end

            -- A linear spline's corners are kept exactly
            local corner = Splynx.linear({ v(0, 0, 0), v(10.3, 0, 0), v(10.3, 0, 7.1) })
            expectNear(Splynx.sampled(corner, 1, true):getPosAtDistance(10.3), v(10.3, 0, 0), "kept corner", 1e-5)
            expect(vector.magnitude(Splynx.sampled(corner, 1):getPosAtDistance(10.3) - v(10.3, 0, 0)) > 0.1, "uniform samples cut the corner")
        end,
    })

    if makeVector3 then
        table.insert(checks, {
            name = "Vector3 inputs are accepted everywhere",
            run = function()
                local V3 = makeVector3
                local hermite = Splynx.hermite({ V3(0, 0, 0), V3(10, 0, 0) }, { V3(10, 0, 0), V3(10, 0, 0) }, V3(1, 2, 3))
                expect(type(hermite:getPos(0.5)) == "vector", "getPos didn't return a vector")
                expectNear(hermite:getPos(0.5), v(6, 2, 3), "midpoint")
                hermite:setPoint(2, V3(20, 0, 0))
                hermite:setVelocity(2, V3(20, 0, 0))
                hermite:setPivot(V3(0, 0, 0))
                hermite:addPoints({ V3(30, 0, 0) }, { V3(10, 0, 0) })
                expectNear(hermite:getPos(1), v(30, 0, 0), "added point")

                local linear = Splynx.linear({ V3(0, 0, 0), V3(10, 0, 0) }, V3(0, 1, 0))
                linear:setPoints({ V3(0, 0, 0), V3(0, 10, 0) })
                expectNear(linear:getPosAtDistance(5), v(0, 6, 0), "linear with Vector3s")
                local sampled = Splynx.sampled(linear, 1)
                sampled:setPivot(V3(0, 0, 0))
                expectNear(sampled:getPosAtDistance(5), v(0, 5, 0), "sampled with Vector3 pivot")

                -- The newer constructors and lookups
                local corner = { V3(0, 0, 0), V3(10, 0, 0), V3(10, 0, 10) }
                for _, spline in { Splynx.rounded(corner, 2, V3(0, 5, 0)), Splynx.filleted(corner, { 0, 2, 0 }, V3(0, 5, 0)) } do
                    expectNear(spline:getJoints()[2], v(8, 5, 0), "rounded with Vector3s")
                    spline:setPoint(3, V3(10, 0, 20))
                    spline:setPivot(V3(0, 0, 0))
                    local position, direction = spline:getOffsetPosAtDistance(4, 1)
                    expectNear(position, v(4, 0, 1), "offset with Vector3s")
                    expectNear(direction, v(1, 0, 0), "direction with Vector3s")
                    local kept = Splynx.sampled(spline, 0.5, true)
                    expectNear(kept:getPosAtDistance(8), v(8, 0, 0), "kept joints with Vector3s", 1e-4)
                end
            end,
        })
    end

    local failures = {}
    for _, check in checks do
        local ok, err = pcall(check.run)
        if not ok then
            table.insert(failures, `{check.name}: {err}`)
        end
    end
    return #checks, failures
end
