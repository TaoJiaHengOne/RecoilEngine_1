--------------------------------------------------------------------------------
--
--  文件:    unitdefs.lua (简化版)
--  简介:    一个简单的unitdef解析器，用于加载所有单位。
--
--------------------------------------------------------------------------------

-- 1. 初始化一个空表，用于存放所有最终的单位定义。
local unitDefs = {}

-- 2. 获取 'units/' 目录下所有以 .lua 结尾的文件列表。
--    这是加载所有独立单位文件的关键步骤。
local unitFiles = VFS.DirList('units/', '*.lua')

-- 3. 遍历文件列表，逐一加载每个单位定义文件。
for i, filename in ipairs(unitFiles) do
    
    -- 使用 pcall (protected call) 来安全地加载文件。
    -- 即使某个单位文件有错误，也不会让整个游戏加载过程崩溃。
    local success, returnedDefs = pcall(VFS.Include, filename)
    
    -- 4. 检查加载是否成功，以及返回的是否是一个有效的表格。
    if success and type(returnedDefs) == 'table' then
        
        -- 5. 如果成功，就遍历返回的表格，并将其中的单位定义合并到主 `unitDefs` 表中。
        --    每个单位文件应该返回一个类似 { unit_name = { ... } } 的表格。
        for unitName, unitData in pairs(returnedDefs) do
            unitDefs[unitName] = unitData
        end
        
    else
        -- 如果加载失败或返回的数据格式不正确，就在日志中记录一个错误。
        Spring.Log('unitdefs.lua', LOG.ERROR, '加载或解析文件时出错: ' .. filename)
    end
end

-- 6. 在所有文件都加载和合并完毕后，返回这个包含所有单位定义的完整表格。
--    引擎将接收这个表格并用它来创建游戏中的单位。
return unitDefs