function Game.Start()

  Spring.Echo("Welcome to my first game! game.lua script is running!")

  local playerList = Spring.GetPlayerList()

  for _, playerID in ipairs(playerList) do
    Spring.AddPlayerResource(playerID, "metal", 1000)
    Spring.AddPlayerResource(playerID, "energy", 1000)
  end

end