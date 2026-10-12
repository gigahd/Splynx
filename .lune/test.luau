-- Runs the correctness checks headless: `lune run test`

local process = require("@lune/process")
local roblox = require("@lune/roblox")

local Splynx = require("../src")
local Checks = require("../tests/Checks")

local count, failures = Checks(Splynx, roblox.Vector3.new)

for _, failure in failures do
    print(`FAIL  {failure}`)
end
print(`{count - #failures}/{count} checks passed`)

if #failures > 0 then
    process.exit(1)
end
