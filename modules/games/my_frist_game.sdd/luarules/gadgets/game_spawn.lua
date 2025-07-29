local gadget = gadget ---@type Gadget
Spring.Echo("game_spawn.lua script is running!")
function gadget:GetInfo()
	return {
		name      = "Spawn",
		desc      = "spawns start unit and sets storage levels",
		author    = "Tobi Vollebregt",
		date      = "January, 2010",
		license   = "GNU GPL, v2 or later",
		layer     = 100,
		enabled   = true  --  loaded by default?
	}
end


if (gadgetHandler:IsSyncedCode()) then
	function gadget:GameStart()
        Spring.Echo("Give all players resources")
		local teamList = Spring.GetTeamList()
		for _, playerID in ipairs(teamList) do
			Spring.AddTeamResource(playerID, "metal", 1000)
			Spring.AddTeamResource(playerID, "energy", 1000)
		end
        Spring.Echo("Spawning 100 units for each team in their start box...")
        -- 遍历所有队伍
        for _, teamID in ipairs(teamList) do
            local startX, startY, startZ = Spring.GetTeamStartPosition(teamID)
            Spring.CreateUnit(
                    "tank_1",                -- 1. 要创建的单位的内部名
                    startX, startY + 50, startZ, -- 2. 创建的随机位置（已抬高）
                    math.random(0, 3),       -- 3. 随机一个朝向 (0-3)
                    teamID,                  -- 4. 单位所属的队伍ID
                    false,                   -- 5. 单位是已建成状态
                    true                     -- 6. 平整地面
                )
        end
           
	end

	function gadget:GameFrame(f)
	end
end




function serialize(obj)
    local seen = {} -- 用于检测循环引用
    
    local function serialize_internal(o, indent)
        -- 基本类型直接转换
        if type(o) ~= "table" then
            if type(o) == "string" then
                return '"' .. tostring(o) .. '"'
            else
                return tostring(o)
            end
        end
        
        -- 检测循环引用
        if seen[o] then
            return "{ ...cycle... }"
        end
        seen[o] = true
        
        local indent_str = indent or ""
        local next_indent_str = indent_str .. "  "
        local parts = {}
        
        for k, v in pairs(o) do
            local key_str = serialize_internal(k, next_indent_str)
            local val_str = serialize_internal(v, next_indent_str)
            table.insert(parts, next_indent_str .. "[" .. key_str .. "] = " .. val_str)
        end
        
        seen[o] = false
        
        if #parts == 0 then
            return "{}"
        end
        
        return "{\n" .. table.concat(parts, ",\n") .. "\n" .. indent_str .. "}"
    end
    
    return serialize_internal(obj, "")
end