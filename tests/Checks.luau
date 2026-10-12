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
                                local ok, err = pcall(spline.getPosAtDistance, spline, distance)
                                expect(not ok and string.find(tostring(err), "enough points") ~= nil, `{kind.name} with {count} points: getPosAtDistance({distance}) gave {err}`)
                            end
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
