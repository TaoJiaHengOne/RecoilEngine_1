-- scripts/game_logic.lua

function gadget:GetInfo()
    return {
        name = "Game Logic",
        desc = "Spawns a unit and handles move commands.",
        author = "You",
        version = "1.0",
        layer = 0,
        enabled = true -- 默认启用这个 Gadget
    }
end

-- 全局变量来存储我们单位的 ID
local ourCubeID = nil

-- [[ 事件: 游戏初始化完成 ]]
-- 这个事件在游戏开始时被引擎调用
function gadget:Game_Start()
    -- 在地图中心 (50%, 50%) 为 0 号玩家创建一个 'cube' 单位
    -- Spring.CreateUnit(unitDefName, position, facing, teamID, flatten)
    ourCubeID = Spring.CreateUnit("cube", Spring.GetMapSize() / 2, "n", 0, true)

    if (ourCubeID) then
        Spring.Echo("方块单位已创建，ID: " .. tostring(ourCubeID))
    else
        Spring.Echo("错误：无法创建方块单位！")
    end
end

-- [[ 事件: 收到指令 ]]
-- 当玩家发出指令时 (如右键点击), 这个事件被调用
-- cmd.id 是指令类型，例如 CMD.MOVE, CMD.ATTACK
-- cmd.params 是指令参数，对于移动指令就是 [x, y, z] 坐标
function gadget:CommandNotify(cmd)
    -- 检查指令是否是移动指令 (CMD.MOVE = 35)
    if (cmd.id == CMD.MOVE and ourCubeID) then
        Spring.Echo("收到移动指令，目标: " .. tostring(cmd.params[1]) .. ", " .. tostring(cmd.params[3]))

        -- 给我们的方块单位下达移动指令
        -- Spring.GiveOrderToUnit(unitID, commandID, params, options)
        Spring.GiveOrderToUnit(ourCubeID, CMD.MOVE, cmd.params, {})
        
        -- 返回 true 表示我们已经处理了这个指令，引擎不需要再做别的了
        return true
    end
    -- 返回 false 让引擎继续处理其他类型的指令
    return false
end