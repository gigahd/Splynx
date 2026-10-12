--!native
--!optimize 2

export type VectorLike = Vector3 | vector

local Vector = {}

function Vector.from(value: VectorLike): vector
    if type(value) == "vector" then
        return (value :: any) :: vector
    end
    local vector3 = (value :: any) :: Vector3
    return vector.create(vector3.X, vector3.Y, vector3.Z)
end

function Vector.fromList(values: { VectorLike }): { vector }
    local out = table.create(#values)
    for i, value in values do
        out[i] = Vector.from(value)
    end
    return out
end

local UP = vector.create(0, 1, 0)
-- A unit direction moving less than this sideways per stud counts as pointing straight up or down
local MIN_SIDEWAYS = 1e-4
Vector.MIN_SIDEWAYS = MIN_SIDEWAYS

-- The unit vector to the right of a unit `direction`, at right angles to `up` too, or zero if `direction` points
-- along `up`
function Vector.rightOf(direction: vector, up: vector?): vector
    local right = vector.cross(direction, up or UP)
    local length = vector.magnitude(right)
    return if length > MIN_SIDEWAYS then right / length else vector.zero
end

-- Replaces each zero vector in `list` with the nearest non-zero one before it, or for those at the start, the first
-- non-zero one
function Vector.fillGaps(list: { vector })
    local first
    for i, value in list do
        if value ~= vector.zero then
            first = first or i
        elseif i > 1 then
            list[i] = list[i - 1]
        end
    end
    for i = 1, (first or 1) - 1 do
        list[i] = list[first]
    end
end

-- The list a batch call fills with `count` values: `out` with anything past `count` removed, or a new list
function Vector.output(out: { vector }?, count: number): { vector }
    if not out then
        return table.create(math.max(count, 0))
    end
    for i = #out, math.max(count, 0) + 1, -1 do
        out[i] = nil
    end
    return out
end

return Vector
