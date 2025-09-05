-- Script to read two integers from serial in format "xxxx,xxxx"
local function init()
    -- Find the first serial port configured for scripting protocol (protocol 28)
    local port = serial:find_serial(0)  -- Using first instance (0)
    
    if not port then
        gcs:send_text(0, "Failed to find serial port")
        return
    end
    
    -- Initialize the port with desired baud rate
    port:begin(9600)
    
    -- Main update function that will be called periodically
    function update()
        -- Check if there are bytes available to read
        local bytes_available = port:available()
        if bytes_available >= 9 then  -- Format "xxxx,xxxx" is 9 characters
            -- Read the string
            local received = port:readstring(9)
             -- Debug print: Show the raw received string
           --- gcs:send_text(0, string.format("Debug - Raw received: '%s'", received))
            
			            -- Find the comma position
            local comma_pos = string.find(received, ",")
            
            if comma_pos == 5 then  -- Comma should be at position 5 for format xxxx,xxxx
                -- Extract the numbers
                local num1 = string.sub(received, 1, 4)
                local num2 = string.sub(received, 6, 9)
                
                -- Convert strings to numbers
                local value1 = tonumber(num1)
                local value2 = tonumber(num2)
                
                -- Verify conversion was successful
                if value1 and value2 then
                    gcs:send_text(0, string.format("Received: %d and %d", value1, value2))
                    -- Add your desired actions with value1 and value2 here
                else
                    gcs:send_text(0, "Failed to convert numbers")
                end
            else
                gcs:send_text(0, "Invalid format received")
            end
        end
        
        return update, 20  -- Schedule next update in 100ms
    end
    
    -- Return the update function and initial delay
    return update, 20
end

-- Register the script
return init()