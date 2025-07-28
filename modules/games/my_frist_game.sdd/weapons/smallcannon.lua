-- weapons/smallcannon.lua

local weaponDef = {
    name = "Small Cannon",
    description = "A basic direct-fire cannon.",

    -- ================= 核心伤害属性 =================
    damage = {
        default = 100, -- 对默认装甲的伤害
        -- 可以为不同装甲类型设置不同伤害，例如：
        -- light = 150,
        -- heavy = 50,
    },
    
    areaOfEffect = 32, -- 爆炸影响范围
    craterMult = 0, -- 不在地面上留弹坑

    -- ================= 射程与开火机制 =================
    range = 450, -- 射程
    reloadTime = 2, -- 装填时间 (秒)
    
    -- ================= 视觉效果 =================
    -- aiteam = "GUN",
    -- cegTag = "SMALL_GUN_MUZZLEFLASH", -- 引用一个开火特效
    
    -- ================= 弹药类型 =================
    weaponType = "Cannon",
    weaponVelocity = 800, -- 炮弹飞行速度

    -- ================= 目标选择 =================
    turret = true, -- 这是一个可以旋转的炮塔武器
    tolerance = 500, -- 开火前允许的瞄准误差
    
    -- ... 其他更多细节参数
}

return { smallcannon = weaponDef }