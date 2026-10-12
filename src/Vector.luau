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

return Vector
