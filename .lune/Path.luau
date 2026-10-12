local Path = {}
Path.__index = Path

type PathData = {
    read index: number,
    read waypoints: { vector },
    read directions: { vector },
    read distances: { number },
    read length: number,
}

export type Path = setmetatable<PathData, typeof(Path)>

local function vector3ToVector(vec: Vector3): vector
    return vector.create(vec.X, vec.Y, vec.Z)
end

function Path.new(index: number, waypoints: { BasePart }): Path
    const positions = table.create(#waypoints) :: { vector }
    const distances = table.create(#waypoints) :: { number }
    const directions = table.create(#waypoints - 1) :: { vector }
    local length = 0
    for i, waypoint in waypoints do
        const position = vector3ToVector(waypoint.Position)
        if i > 1 then
            const temp = position - positions[i - 1]
            directions[i - 1] = vector.normalize(temp)
            length += vector.magnitude(temp)
        end
        positions[i] = position
        distances[i] = length
    end
    local self: PathData = {
        index = index,
        waypoints = positions,
        directions = directions,
        distances = distances,
        length = length,
    }
    return setmetatable(self, Path)
end

function Path.Get(self: Path, index: number): (vector, vector?)
    assert(
        index > 0 and index <= #self.waypoints,
        `Index out of bounds, expected index between 1 and {#self.waypoints}, got {index}`
    )
    return self.waypoints[index], self.directions[index]
end

function Path.GetWaypointDistance(self: Path, waypointIndex: number): number
    return self.distances[waypointIndex]
end

function Path.MoveAlong(self: Path, distance: number, waypointIndex: number): (vector, vector?, number)
    const waypoints = self.waypoints
    const distances = self.distances
    const directions = self.directions
    const waypointCount = #waypoints
    if distance >= self.length then
        return waypoints[waypointCount], nil, waypointCount
    end
    if distance <= 0 then
        return waypoints[1], directions[1], 1
    end

    local index = math.clamp(waypointIndex, 1, waypointCount - 1)
    while distance >= distances[index + 1] do
        index += 1
    end
    while distance < distances[index] do
        index -= 1
    end

    const direction = directions[index]
    return waypoints[index] + direction * (distance - distances[index]), direction, index
end

return Path
