---- paramètre de l'uav


---correcteur sur le pitch
local Kp_pitch = 2 ---GAIN A MODIIER POUR LE PITCH P
local Ki_pitch = 0.4 ---GAIN A MODIIER POUR LE PITCH I
local Integral_pitch = 0


----correcteur sur le roll
local Kp_roll = 3 ---GAIN A MODIIER POUR LE ROLL P
local Ki_roll = 0.550 ---GAIN A MODIIER POUR LE PITCH I
local Integral_roll = 0

----correcteur sur le roll
local Kp_rudder = 3 ---GAIN A MODIIER POUR LE rudder P
local Ki_rudder = 0.550 ---GAIN A MODIIER POUR LE rudder I
local Integral_rudder = 0

local K_correct = 100  -- Coefficient de correction sortie PWM
local pitch_trim = 0  --trim du pitch sur le pwm
local roll_trim = 0 --trim du roll sur le pwm
local rudder_trim = 0  --trim du rudder sur le pwm

local Imax = 500/K_correct
local current_roll = 0

local trim_left_aileron = 1500 - param:get('SERVO1_TRIM')
local trim_right_aileron = 1500 - param:get('SERVO2_TRIM')
local trim_up_rudder = 1500 - param:get('SERVO3_TRIM')
local trim_down_rudder = 1500 - param:get('SERVO4_TRIM')
-----revoir qui est le right et qui est le left aileron
local upper_left_aileron = param:get('SERVO1_MAX')
local lower_left_aileron = param:get('SERVO1_MIN')

local upper_right_aileron = param:get('SERVO2_MAX')
local lower_right_aileron = param:get('SERVO2_MIN')

local upper_up_rudder = param:get('SERVO3_MAX')
local lower_up_rudder= param:get('SERVO3_MIN')

local upper_down_rudder = param:get('SERVO4_MAX')
local lower_down_rudder = param:get('SERVO4_MIN')


local target_roll = 0 -- target_roll reste toujours à 0 normalement
local target_pitch = 0 --paramètre de consigne de piquer (pitch)
local init_yaw = 0

local current_pitch = 0  ---orientation du drone dans l'espace
local current_yaw = 0
    -- Find the first serial port configured for scripting protocol (protocol 28)
local port = serial:find_serial(0)  -- Using first instance (0)
    
    if not port then
        gcs:send_text(0, "Failed to find serial port")
        return
    end
	   -- Initialize the port with desired baud rate
    port:begin(9600)

local optical_loss = 0
local start_switch_num = 5 ---A MODIFIER LE NUMERO DU CHANNEL DE LA TELECOMMANDE QUI DECLENCHE LE SCRIPT DE DESCENTE
local start_switch = 1000 ---initialisation du switch de déclenchement
local last_switch_high = 0 ---initialisation pour lecture changement position

local err_targ_pitch = 0
local err_targ_rudder = 0
local script_start = 1

local dt = 20  -- intervalle de temps en millisecondes

-- Correcteur PI pour le pitch
function pi_corrector_pitch(err_target)
    local P = Kp_pitch * err_target
    local I = Ki_pitch * err_target * 0.001 * dt + Integral_pitch

    Integral_pitch = Integral_pitch + err_target * 0.001 * dt  -- Mise à jour de l'intégral

    if Integral_pitch > Imax then
        Integral_pitch = Imax
    elseif Integral_pitch < -Imax then
        Integral_pitch = -Imax
    end

    local output_pi = P + I
    return output_pi
end

-- Correcteur PI pour le roll
function pi_corrector_roll()
	local err_target = target_roll - current_roll
    local P = Kp_roll * err_target
    local I = Ki_roll * err_target *0.001* dt + Integral_roll

    Integral_roll = Integral_roll + err_target * 0.001*dt  -- Mise à jour de l'intégral

    if Integral_roll > Imax then
        Integral_roll = Imax
    elseif Integral_roll < -Imax then
        Integral_roll = -Imax
    end

    local output_pi = P + I
    return output_pi
end

function pi_corrector_rudder(err_target)
    local P = Kp_rudder * err_target
    local I = Ki_rudder * err_target * 0.001 * dt + Integral_pitch

    Integral_rudder = Integral_rudder + err_target * 0.001 * dt  -- Mise à jour de l'intégral

    if Integral_rudder > Imax then
        Integral_rudder = Imax
    elseif Integral_rudder < -Imax then
        Integral_rudder = -Imax
    end

    local output_pi = P + I
    return output_pi
end

function scale_output(out_pi, Trim)
    -- Scaling the output using K_correct and clamping between 1210 and 1780 --- PEUT ETRE MODIFIER MAIS PAS CONSEILLE
    local scaled = 1500 + K_correct * out_pi + Trim
    if scaled > 1900 then
        scaled = 1900
    elseif scaled < 1100 then
        scaled = 1100
    end
    return scaled
end
	---peut être mettre une correction sur la vitesse du drone

function limit_value(value, lower_bound, upper_bound)
    if value > upper_bound then
        value = upper_bound
    elseif value < lower_bound then
        value = lower_bound
    end
    return value
end

function attitude_control(err_target_pitch, err_target_rudder)
	local err_pitch = err_target_pitch * math.pi/180
	local err_rudder = err_target_rudder * math.pi/180 ---normalisation en radians

    local output_pi_pitch = pi_corrector_pitch(err_pitch)
    local output_pi_roll = pi_corrector_roll()
    local output_pi_rudder = pi_corrector_rudder(err_rudder)
   
    -- Scale and saturate the outputs
    local scaled_pitch = scale_output(output_pi_pitch, pitch_trim)  
    local scaled_roll = scale_output(output_pi_roll, roll_trim)
	local scaled_rudder = scale_output(output_pi_rudder, rudder_trim)
	
    	pwm_pitch = math.floor(scaled_pitch)    
    	pwm_roll = math.floor(scaled_roll)
		pwm_rudder = math.floor(scaled_rudder)
		
		gcs:send_text(6, pwm_roll)
		gcs:send_text(6, pwm_pitch)
	local output_roll_left = 3000 - (1500 - pwm_pitch + pwm_roll - trim_left_aileron)
    output_roll_left = limit_value(output_roll_left, lower_left_aileron, upper_left_aileron)
    
    local output_roll_right = 4500 - (pwm_pitch + pwm_roll - trim_right_aileron)
    output_roll_right = limit_value(output_roll_right, lower_right_aileron, upper_right_aileron)
    
    local output_rudder_up = 3000 - (pwm_roll + (-pwm_rudder + 1500) + trim_up_rudder)
    output_rudder_up = limit_value(output_rudder_up, lower_up_rudder, upper_up_rudder)
    -----mettre pwm rudder
    local output_rudder_down = 3000 -(pwm_roll + (-1500 + pwm_rudder) - trim_down_rudder)
    output_rudder_down = limit_value(output_rudder_down, lower_down_rudder, upper_down_rudder)
	
	
	SRV_Channels:set_output_pwm_chan_timeout(0, output_roll_left,500)	--SERVO1
	SRV_Channels:set_output_pwm_chan_timeout(1, output_roll_right,500)	--SERVO2
	
	SRV_Channels:set_output_pwm_chan_timeout(2, output_rudder_up,500) --SERVO3
	SRV_Channels:set_output_pwm_chan_timeout(3, output_rudder_down,500) --SERVO4
    
end

function init_uart()
    -- Find the first serial port configured for scripting protocol (protocol 28)
    port = serial:find_serial(0)  -- Using first instance (0)
    
    if not port then
        gcs:send_text(0, "Failed to find serial port")
        return
    end
	   -- Initialize the port with desired baud rate
    port:begin(115200)
 end   

function read_UART()
    local bytes_available = port:available()
    if bytes_available >= 14 then  -- Format "xxxx,xxxx,xxxx" is 14 characters
        -- Read the string
        local received = port:readstring(14)
        gcs:send_text(6, string.format("Debug - Raw received: '%s'", received))
        -- Find the comma positions
		local first_comma = string.find(received, ",")
		local second_comma = string.find(received, ",", first_comma + 1)

        if first_comma == 5 and second_comma == 10 then  -- Commas should be at position 5 and 10
            -- Extract the numbers
            local num1 = string.sub(received, 1, 4)
            local num2 = string.sub(received, 6, 9)
            local num3 = string.sub(received, 11, 14)
            
            -- Convert strings to numbers
            local value1 = tonumber(num1)
            local value2 = tonumber(num2)
            local value3 = tonumber(num3)
            
            -- Verify conversion was successful
            if value1 and value2 and value3 then
                --gcs:send_text(0, string.format("Received: %d, %d, %d", value1, value2, value3))
                return value1, value2, value3
            else
                gcs:send_text(6, "Failed to convert numbers")
                return 0, 0, 1
            end
        else
            gcs:send_text(6, "Invalid format: commas not at expected positions")
            return 0, 0, 2
        end
    end
    return 0, 0, 2  -- Return defaults if no data available
end

gcs:send_text(6, "Script STARTED")
function update()
---mise à jour des angles
    current_roll = ahrs:get_roll()
    local instant_pitch =  ahrs:get_pitch()
	local instant_yaw = ahrs:get_yaw()

	
	start_switch = rc:get_pwm(start_switch_num)
		
	--------ETAT de réinitialisation des états-----------------
----mettre une detection que si le switch est activé puis ne l'ai plus il se met en mode RTL
	if last_switch_high>1550 and start_switch < 1550 or script_start == 1 then
		Integral_pitch=0
		Integral_roll=0
		Integral_rudder = 0
		script_start = 0
		gcs:send_text(6, "Script Running")
		init_yaw = ahrs:get_yaw()
		init_uart()
    end
	
	local erreur_pitch = 0
	local erreur_rudder = 0
	erreur_pitch, erreur_rudder, optical_loss = read_UART()
		gcs:send_text(6, string.format("erreur_pitch: '%d'", erreur_pitch))
		gcs:send_text(6, string.format("erreur_rudder: '%d'", erreur_rudder))
		gcs:send_text(6, string.format("indice_erreur: '%d'", optical_loss))
		
	last_switch_high = start_switch ---mise en mémoire dernier switch
	
 if start_switch > 1500 then ----plus de FAILSAFE il tente coute que coute de se poser

	if optical_loss <2 then
		err_targ_pitch = erreur_pitch
		err_targ_rudder = erreur_rudder ---maintient position servo entre deux msg reçu
	end
	
	if optical_loss == 0 then
		current_pitch =  ahrs:get_pitch()
		current_yaw = ahrs:get_yaw()
	end
	target_roll = 0
	if optical_loss == 1 then
		err_targ_pitch = current_pitch - instant_pitch
		err_targ_rudder = current_yaw - instant_yaw
	end

    attitude_control(err_targ_pitch,err_targ_rudder)

 end	
    
	return update, dt

end
return update, dt


---mettre un protected wrapper
