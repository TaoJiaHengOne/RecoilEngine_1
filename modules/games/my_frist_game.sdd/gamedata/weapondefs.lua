--------------------------------------------------------------------------------
--
--  文件:    weapondefs.lua (简化版)
--  简介:    一个简单的 weapondef 解析器，用于加载所有武器。
--
--------------------------------------------------------------------------------

-- 1. 初始化一个空表，用于存放所有最终的武器定义。
local weaponDefs = {}

-- 2. 获取 'weapons/' 目录下所有以 .lua 结尾的文件列表。
local weaponFiles = VFS.DirList('weapons/', '*.lua')

-- 3. 遍历文件列表，逐一加载每个武器定义文件。
for _, filename in ipairs(weaponFiles) do
    
    -- 使用 pcall (protected call) 来安全地加载文件。
    -- 即使某个武器文件有错误，也不会让整个游戏加载过程崩溃。
    local success, returnedDefs = pcall(VFS.Include, filename)
    
    -- 4. 检查加载是否成功，以及返回的是否是一个有效的表格。
    if success and type(returnedDefs) == 'table' then
        
        -- 5. 如果成功，就遍历返回的表格，并将其中的武器定义合并到主 `weaponDefs` 表中。
        --    每个武器文件应该返回一个类似 { weapon_name = { ... } } 的表格。
        for weaponName, weaponData in pairs(returnedDefs) do
            weaponDefs[weaponName] = weaponData
        end
        
    else
        -- 如果加载失败或返回的数据格式不正确，就在日志中记录一个错误。
        Spring.Log('weapondefs.lua', LOG.ERROR, '加载或解析文件时出错: ' .. filename)
    end
end

-- 6. 在所有文件都加载和合并完毕后，返回这个包含所有武器定义的完整表格。
--    这个表格会被 `defs.lua` 接收，并最终提交给引擎。
return weaponDefs