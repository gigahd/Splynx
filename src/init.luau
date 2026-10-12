--!native
--!optimize 2

--[=[
    @class Splynx

    Splines built for fast lookups. There are three kinds:

    | Kind | Made by | Shape | Editable |
    | --- | --- | --- | --- |
    | [LinearSpline] | [Splynx.linear] | Straight segments between the points | Yes |
    | [CubicSpline] | [Splynx.bezier], [Splynx.hermite], [Splynx.cardinal], [Splynx.catmullRom], [Splynx.bSpline], [Splynx.rounded], [Splynx.filleted] | Smooth curves | Yes |
    | [SampledSpline] | [Splynx.sampled] | Straight segments between samples along another spline | No |

    Lookups are prioritised over edits: everything that speeds up a lookup is precomputed, and rebuilt when the
    spline changes. Run `lune run bench` for current numbers.

    ### Points and the pivot

    Points are relative to the spline's pivot, and every position a spline returns has the pivot applied. Moving
    the pivot moves the whole spline without rebuilding anything.

    ### Alpha and distance

    `getPos(alpha)` splits alpha evenly across a spline's curves, so it speeds up and slows down along curves of
    different lengths. For a constant speed, work in distances: `getPosAtDistance`, `getUniformPos` and
    `sampleUniform`.

    Every distance lookup has variants that also give the direction of travel (`getPosAndDirectionAtDistance`),
    a position off to the side of the spline (`getOffsetPosAtDistance`), or many lookups at once written into
    lists you keep (`getPosAtDistances`).

    ### Hints

    Distance lookups take an optional hint and return a new one. Passing back the previous call's hint starts the
    next lookup where the last one ended, which makes lookups for something moving along the spline cheaper. Any
    hint gives the same result, so a stale one is fine; leave it out for lookups that jump around.

    ```lua
    local spline = Splynx.catmullRom({ Vector3.new(0, 0, 0), Vector3.new(10, 5, 0), Vector3.new(20, 0, 0) })

    local distance, hint = 0, nil
    RunService.Heartbeat:Connect(function(deltaTime)
        distance = (distance + 12 * deltaTime) % spline:getLength()
        local position
        position, hint = spline:getPosAtDistance(distance, hint)
        part.Position = position
    end)
    ```

    ### Vectors

    Inputs accept `Vector3` or `vector`, and outputs are `vector`. On Roblox the two are the same value, but the
    type checker treats them as different types, so outputs may need a cast where a `Vector3` is expected.
]=]
local CubicSpline = require("@self/CubicSpline")
local LinearSpline = require("@self/LinearSpline")
local SampledSpline = require("@self/SampledSpline")
local Vector = require("@self/Vector")

local Bezier = require("@self/Builders/Bezier")
local BSpline = require("@self/Builders/BSpline")
local Cardinal = require("@self/Builders/Cardinal")
local Hermite = require("@self/Builders/Hermite")
local Rounded = require("@self/Builders/Rounded")

--[=[
    @within Splynx
    @type VectorLike Vector3 | vector

    Anything Splynx accepts as a position or velocity.
]=]
export type VectorLike = Vector.VectorLike
export type LinearSpline = LinearSpline.LinearSpline
export type CubicSpline = CubicSpline.CubicSpline
export type SampledSpline = SampledSpline.SampledSpline
export type Segments = SampledSpline.Segments
--[=[
    @within Splynx
    @type Spline LinearSpline | CubicSpline | SampledSpline

    Any spline.
]=]
export type Spline = LinearSpline | CubicSpline | SampledSpline

local Splynx = {}

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param pivot VectorLike? -- Defaults to the origin.
    @return LinearSpline

    Straight segments between the points. Needs at least two points to make a segment.
]=]
function Splynx.linear(points: { VectorLike }?, pivot: VectorLike?): LinearSpline
    return LinearSpline.new(points or {}, pivot)
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    Cubic Bezier curves: points 1-4 make the first curve, 4-7 the second, and so on, so it passes through every
    third point. Points past the last complete curve are kept but ignored.
]=]
function Splynx.bezier(points: { VectorLike }?, pivot: VectorLike?): CubicSpline
    return CubicSpline.new(Bezier, points or {}, pivot)
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param velocities {VectorLike}? -- One per point. Missing ones are zero.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    Passes through every point, leaving each one with its velocity. Change velocities with
    [CubicSpline:setVelocity].
]=]
function Splynx.hermite(points: { VectorLike }?, velocities: { VectorLike }?, pivot: VectorLike?): CubicSpline
    return CubicSpline.new(Hermite, points or {}, pivot, velocities or {})
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param scale number -- How strongly the curve follows its neighbours; 0.5 is Catmull-Rom.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    Passes through every point, with velocities of `(next - previous) * scale`. The end points mirror their only
    neighbour. Change the scale with [CubicSpline:setScale].
]=]
function Splynx.cardinal(points: { VectorLike }?, scale: number, pivot: VectorLike?): CubicSpline
    return CubicSpline.new(Cardinal, points or {}, pivot, nil, scale)
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    A [Splynx.cardinal] spline with a scale of 0.5: passes through every point with natural-looking curves.
]=]
function Splynx.catmullRom(points: { VectorLike }?, pivot: VectorLike?): CubicSpline
    return Splynx.cardinal(points, 0.5, pivot)
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    A uniform cubic B-spline: smooth everywhere, including across joints, but it only passes near its points.
    Needs at least four points.
]=]
function Splynx.bSpline(points: { VectorLike }?, pivot: VectorLike?): CubicSpline
    return CubicSpline.new(BSpline, points or {}, pivot)
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param reach number | {number} -- How far either side of every corner the rounding starts and ends, or one per point. Missing ones are 0.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    Straight lines between the points, with the corner at each point rounded off by a circular arc. The arc starts
    `reach` studs before the corner and ends `reach` studs after it, whatever the angle, so sharper turns get
    tighter arcs. Unlike [Splynx.catmullRom], the straight parts stay straight, so it suits paths laid out from
    waypoints. The ends and corners with a reach of 0 stay sharp. For arcs of a set radius instead, use
    [Splynx.filleted].

    Where the arcs at both ends of a line would need more of it than there is, both shrink in proportion until
    they meet, so a reach is the most a corner gets. A path that doubles back on itself turns sharply. Arcs are
    cubic approximations, within about 0.03% of their radius for a right angle.

    Its curves alternate between lines and arcs, starting and ending with a line. Change reaches with
    [CubicSpline:setRadius].

    ```lua
    local path = Splynx.rounded({
        Vector3.new(0, 0, 0),
        Vector3.new(40, 0, 0),
        Vector3.new(40, 0, 30),
    }, 2)
    ```
]=]
function Splynx.rounded(points: { VectorLike }?, reach: number | { number }, pivot: VectorLike?): CubicSpline
    return CubicSpline.new(Rounded.byReach, points or {}, pivot, nil, nil, reach)
end

--[=[
    @within Splynx
    @param points {VectorLike}? -- Relative to the pivot.
    @param radius number | {number} -- The arc radius of every corner, or one per point. Missing ones are 0.
    @param pivot VectorLike? -- Defaults to the origin.
    @return CubicSpline

    Like [Splynx.rounded], but each corner is an arc of `radius`. An arc of the same radius needs more room the
    sharper the turn: it starts `radius * tan(turn / 2)` studs before the corner, the radius at a right angle and
    about 3.7 times it at a 150 degree turn. Where there isn't that much room, the arc shrinks to fit, which also
    shrinks its radius.
]=]
function Splynx.filleted(points: { VectorLike }?, radius: number | { number }, pivot: VectorLike?): CubicSpline
    return CubicSpline.new(Rounded.byRadius, points or {}, pivot, nil, nil, radius)
end

--[=[
    @within Splynx
    @param source LinearSpline | CubicSpline -- The spline to copy.
    @param spacing number -- Studs between samples, along the spline.
    @param keepJoints boolean? -- Put a sample on every joint, so sharp corners stay sharp.
    @return SampledSpline
    @error "Spacing must be more than 0" -- If `spacing` isn't positive.
    @error "Spline doesn't have enough points to make a curve" -- If `source` has no curves.

    A static copy of `source` as straight segments `spacing` studs apart along it, for the fastest distance
    lookups. Smaller spacing follows curves more closely. It doesn't follow later edits to `source`; sample it
    again after changing it.

    Samples that land either side of a sharp corner cut across it. With `keepJoints`, every joint (see
    `getJoints`) gets a sample, and each curve is split into equal segments no longer than `spacing`, so a
    linear spline's corners and a rounded spline's straight lines come out exact. Lookups cost the same either
    way.
]=]
function Splynx.sampled(source: LinearSpline | CubicSpline, spacing: number, keepJoints: boolean?): SampledSpline
    return SampledSpline.new(source, spacing, keepJoints)
end

return Splynx
