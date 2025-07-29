	
local moveDatas = {
    -- 科尔加普、臂 Beaver（或 “臂海狸”，可能是带机械臂的单位）、
    -- 臂 Mar（或 “臂蝾螈”，“Mar” 可能关联 “marine”）、科尔箭（“arrow” 对应箭，可能是快速单位）、
    -- 臂 prow（或 “臂舰首”，可能是突击型单位）、科尔海豹、科尔萨拉（可能是水生单位）、科尔麝鼠（小型单位）、
    -- 臂鳄（鳄鱼形态机械单位）、臂钳（带钳状武器的单位）、科尔 intr（可能是 “科尔潜伏者”，“intr” 关联 “intruder”）、
    -- 腿部辅助无人机（陆基型）、科尔辅助无人机、臂部辅助无人机、腿水獭（腿部形态类似水獭的单位）、科尔幻影（隐形 / 幻象单位）
    ATANK3 = {
		crushstrength = 30,
		depthmod = 0,
		footprintx = 2,
		footprintz = 2,
		maxslope = 36,
		slopeMod = 18,
		maxwaterdepth = 5000,
		maxwaterslope = 80,
	},
}

local defs = {} -- 1. 创建一个空列表（在Lua中也叫table）

-- 2. 遍历上面定义好的、名为 moveDatas 的总表
for moveName, moveData in pairs(moveDatas) do 
	
    -- 3. 对每一个移动类型，都强制加上或覆盖这些通用属性
	moveData.heatmapping = true
	moveData.allowRawMovement = true
	moveData.allowTerrainCollisions = false

    -- 4. 把这个移动类型的名字（比如 "TANK2"）也作为一个属性加进去
	moveData.name = moveName
	
    -- 5. 如果移动类型的名字包含 "BOT" 并且它有最大坡度属性，就进行一项特殊处理
	if moveName and string.find(moveName, "BOT") and moveData.maxslope then
		moveData.slopemod = 4 -- 给所有机器人设置一个特定的坡度修正值
	end

	-- 6. 将这个被处理过的 moveData，作为新的一项，添加到 defs 列表中
	defs[#defs + 1] = moveData
end

for k, v in pairs(defs) do
    Spring.Echo("load move def: " .. v.name)
end

return defs -- 7. 返回这个最终处理好的列表