# Splynx

Fast splines for Roblox. Linear, Bezier, Hermite, Catmull-Rom, cardinal and B-splines, with constant-speed movement
along any of them.

## Install

With [Wally](https://wally.run):

```toml
[dependencies]
Splynx = "gigahd/splynx@0.1.0"
```

## Use

```lua
local Splynx = require(path.to.Splynx)

local spline = Splynx.catmullRom({
    Vector3.new(0, 0, 0),
    Vector3.new(10, 5, 0),
    Vector3.new(20, 0, 0),
})

-- Move a part along it at 12 studs per second
local distance, hint = 0, nil
RunService.Heartbeat:Connect(function(deltaTime)
    distance = (distance + 12 * deltaTime) % spline:getLength()
    local position
    position, hint = spline:getPosAtDistance(distance, hint)
    part.Position = position
end)
```

Need the fastest lookups and won't edit the spline? `Splynx.sampled(spline, 0.5)` makes a static copy with constant-time
distance lookups.

Laying a path out from waypoints? `Splynx.rounded(points, 2)` keeps the straight parts straight and rounds each
corner, starting 2 studs before it and ending 2 studs after. For many things moving along one spline,
`getPosAtDistances` fills positions, directions and sideways offsets for all of them in one call, into tables you reuse.

## Develop

Tools are managed with [Rokit](https://github.com/rojo-rbx/rokit): `rokit install`.

- `lune run test`: correctness checks
- `lune run bench`: benchmarks
- `rojo serve test.project.json`: the same checks plus live visualizations in Studio (press F8 and drag the points)
- `npx moonwave dev`: the documentation site

## License

MIT

## [Documentation](https://gigahd.github.io/Splynx/)
