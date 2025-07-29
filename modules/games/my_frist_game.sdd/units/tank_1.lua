Spring.Echo("exec units/tank_1.lua")

local def = {
    tank_1 = {
        name = "tank_1",
        objectname = "tank_1.blend.s3o",
        description = "A simple demo unit.", -- 单位描述
          -- ================= 建造与资源 =================
        buildCostEnergy = 100, -- 建造所需能源
        buildCostMetal = 100, -- 建造所需金属
        buildTime = 10, -- 建造时间（秒）

        -- ================= 战斗属性 =================
        maxDamage = 1000, -- 最大生命值
        -- ================= 移动属性 (这是实现“移动”的关键) =================
        -- 引擎会根据这些值自动处理移动，你不需要写代码
        canMove = true, -- 该单位是否可以移动
        movementClass = "ATANK3", -- 移动类型，决定了地形适应性
        maxVelocity = 2.5, -- 最大速度 (单位：elmos/秒)
        acceleration = 0.1, -- 加速度
        brakeRate = 0.2, -- 刹车率
        turnRate = 400, -- 转向速度
         -- ================= 视野与雷达 =================
        sightDistance = 500, -- 视野范围
        radarDistance = 600, -- 雷达范围

        -- ================= 攻击武器 (这是实现“攻击”的关键) =================
        weapons = {
            {
                name = "SmallCannon", -- 引用一个武器定义 (通常在 `weapons` 文件夹里)
                mainDir = "0 0 1", -- 武器开火方向 (前)
                maxAngleDif = 180, -- 武器可转动的最大角度
            },
        },
    }
}

return def