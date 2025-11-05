thread.start(function()
    pio.pin.setdir(pio.OUTPUT, pio.GPIO32)
    pio.pin.setpull(pio.PULLUP, pio.GPIO32)
    
    while true do
        pio.pin.setval(1, pio.GPIO32)
        tmr.delayms(1000)
        pio.pin.setval(0, pio.GPIO32)
        tmr.delayms(1000)
    end
end)
