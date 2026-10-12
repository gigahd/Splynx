-- Runs the same checks as `lune run test`, but inside Roblox

local ReplicatedStorage = game:GetService("ReplicatedStorage")

local Splynx = require(ReplicatedStorage:WaitForChild("Splynx"))
local Checks = require(ReplicatedStorage:WaitForChild("SplynxChecks"))

local count, failures = Checks(Splynx, Vector3.new)

for _, failure in failures do
    warn(`[Splynx] FAIL  {failure}`)
end
print(`[Splynx] {count - #failures}/{count} checks passed`)
