-- Flight control for Spaceship v0.1 by Ace
-- Designed for Hi!SuperFlat modpack
-- waypoint for autopilot
local waypoint = {
	targetid = 1,
	targets = {
		{x = 0, y = -20, z = 0, yaw = 0, comment = "home"},
	},
}
local prev_targetid = 1

-- properties of the ship
local properties = {
	mode = 2, -- 1: spaceship; 2: starship (no roll/pitch)
	mode_name = {"Spaceship mode", "Starship mode"},
	tick_mode = 2, -- 1: CC ticks; 2: physics_tick
	cruiseY = 300,
	placementFace = "north",
    K = {
		x={P=2, I=0, D=0.2},
		y={P=2, I=1, D=0.2},
		z={P=2, I=0, D=0.2},
		omega={P=2, I=0, D=0.2},
		level={P=5, I=0, D=0.4},
	},
	Vmax = {
		x = 35,
		y = 35,
		z = 35,
	},
	Omega_max = {
		x = 1.5,
		y = 0.7,
		z = 0.7,
	},
	gravity_mult = {0, 0},
	NESWYaw = { -- subject to initial placementFace
		N = 0,
		E = -math.pi/2,
		S = -math.pi,
		W = math.pi/2,
	},
	name = "ZHDDSFJBJLINNB",
}

-- Display setting
local Waypoint_side = "left"
local Data_side = "right"

-- Display colors
local Color_auto = 0x8 -- lightblue
local Color_lock = 0x4000  -- red
local Color_level = 0x2000  -- green
local Color_default = 0x1 -- white
local Color_highlight = 0x8000 -- black
local Color_bg_highlight = 0x1 -- white
local Color_bg_default = 0x8000 -- white

-- hologram
local hologram_eye_offset = 0.12

-- initialize engine
local engine = peripheral.find("EngineController") -- driven by EngineController from void power mod
local mass_rpm_ratio = 20000 -- mass/mass_rpm_ratio = RPM required
------------------------------------------------


-- controller ----------------------------------
local controller = {
    joy = nil,
    flag = false
}

local prevStart = false -- record previous status of start button, for locking position
local prevBack = false -- record previous status of end button, for auto-trimming
local prevLeftJoyClick = false -- record previous status of left stick, for autopilot

controller.defaultOutput = function()
	return {
		LeftStick      = { x = 0,  y = 0},
		RightStick     = { x = 0, y = 0 },
		LeftJoyClick   = false,
		RightJoyClick  = false,
		LB             = false,
		RB             = false,
		LT             = 0,
		RT             = 0,
		back           = false,
		start          = false,
		up             = false,
		down           = false,
		left           = false,
		right          = false,
	}
end

controller.getInput = function()
	if not controller.joy then -- controller not setup yet
		controller.joy = peripheral.find("tweaked_controller") -- setup controller
	end
	if controller.joy.hasUser() then
		return {
			--push right = positive; push left = negative; push up = positive; push down = negative
			LeftStick      = { x = controller.joy.getAxis(1),  y = controller.joy.getAxis(2)},
			--push right = positive; push left = negative; push up = positive; push down = negative
			RightStick     = { x = controller.joy.getAxis(3), y = controller.joy.getAxis(4) },
			LeftJoyClick   = controller.joy.getButton(10),
			RightJoyClick  = controller.joy.getButton(11),
			LB             = controller.joy.getButton(5),
			RB             = controller.joy.getButton(6),
			LT             = controller.joy.getAxis(5),
			RT             = controller.joy.getAxis(6),
			back           = controller.joy.getButton(7),
			start          = controller.joy.getButton(8),
			up             = controller.joy.getButton(12),
			down           = controller.joy.getButton(14),
			left           = controller.joy.getButton(15),
			right          = controller.joy.getButton(13),
		}
	else
		return controller.defaultOutput()
	end
end

controller.AllowedUser = {
	"115d5fee-cab8-4896-9a74-d76630d998cc", -- admin, Ace_Volca
      "4d92b9f2-584e-4ad1-a497-9d5aeaa7b07a", -- owner, ZH
}
controller.AllowedUserName = {
	"AceVolca",
  "ZH",
}
------------------------------------------------


-- global functions ---------------------------
function TransposeMat(M)
	return {
	  {M[1][1], M[2][1], M[3][1]},
	  {M[1][2], M[2][2], M[3][2]},
	  {M[1][3], M[2][3], M[3][3]},
	}
end

function MulMatVec(M, v) -- return Matrix.dot(vector)
	return {
	  x = M[1][1]*v.x + M[1][2]*v.y + M[1][3]*v.z,
	  y = M[2][1]*v.x + M[2][2]*v.y + M[2][3]*v.z,
	  z = M[3][1]*v.x + M[3][2]*v.y + M[3][3]*v.z,
	}
end

function QuatMultiply(q1, q2)
    local newQuat = {}
    newQuat.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w
    newQuat.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x
    newQuat.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y
    newQuat.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z
    return newQuat
end

function QuatToRotMat(q) -- transform quaternion to rotation matrix
	local w,x,y,z = q.w, q.x, q.y, q.z
	return {
		{1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)},
		{  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)},
		{  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)},
	}
end

function Scale2Max(v, max_v) -- scale a value to its max, max_v has to be positive
	if math.abs(v) > max_v then
		if v < 0 then
			return -max_v
		else
			return max_v
		end
	else
		return v
	end
end

-- transform vector in body coordinate to world coordinate
function BodyToWorld(v_body, quat)
	local rot_mat = QuatToRotMat(quat)
	return MulMatVec(rot_mat, v_body)
end

-- transform vector in world coordinate to body coordinate
function WorldToBody(v_world, quat)
	local rot_mat = QuatToRotMat(quat)
	local rot_mat_T = TransposeMat(rot_mat)
	return MulMatVec(rot_mat_T, v_world)
end

-- normalize a vector, cannot deal with 0
function NormVector(v)
	local norm = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
	if norm < 0.001 then
		return {x=0, y=0, z=0}
	else
		return {x=v.x/norm, y=v.y/norm, z=v.z/norm}
	end
end

-- return L2 Norm of a vector (vx,vy,vz)
function L2Norm(v)
	return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
end

-- wrap angle in rad to [-pi, pi]
function WrapAngle(a)
	while a >  math.pi do a = a - 2*math.pi end
	while a < -math.pi do a = a + 2*math.pi end
	return a
end

-- get Euler from body quaternion
-- facing -z (forward); initial roll=0, clockwise +roll, counter-clockwise -roll
-- initial pitch = 0, lift up +pitch, dive down -pitch
-- turn right -yaw, turn left +yaw
-- facing south = -pi(+pi), west = +pi/2, north = 0,  east = -pi/2, subject to initial placementFace
local roll_offset

function Quat2Euler_Vector(q_orig)
    local w, x, y, z = q_orig.w, q_orig.x, q_orig.y, q_orig.z

    -- 1) rotation matrix from ship quat（3×3）
    local R = QuatToRotMat(q_orig)
    -- R[i][j] ith row jth col

    -- 2) 3rd col of R  (R[1][3], R[2][3], R[3][3]) is Body‐forward in world position
    --    Body-forward = (0,0,-1)，in world coordinate (-R[1][3], -R[2][3], -R[3][3])
    local Fx = -R[1][3]
    local Fy = -R[2][3]
    local Fz = -R[3][3]

    -- 3) body-forward projection on world XZ plane
    local horiz_len = math.sqrt(Fx*Fx + Fz*Fz)

    -- 4) use atan2 to obtain signed pitch (unwrapped)
    --    when lift up, Fy>0 ⇒ pitch>0；when push down, Fy<0 ⇒ pitch<0。
    local pitch = math.atan2(Fy, horiz_len)
	local body_up_y = R[2][2]  -- body y's projection on world y

    -- if body_up_y < 0，then upside down，rotate pitch by 180°
    if body_up_y < 0 then
        pitch = WrapAngle(pitch + math.pi) -- when pitch ~ pi/2, sign flips
    end

    local z_rh = -z -- 1) reverse z to use formula for right-handed system
    local siny_cosp = 2 * (w * y + x * z_rh)
    local cosy_cosp = 1 - 2 * (x*x + y*y)
	-- 5) Yaw   = atan2( 2*(w*y + x*z_rh),  1 - 2*(x^2 + y^2) )
    local yaw = math.atan2(siny_cosp, cosy_cosp)

    local sinr_cosp = 2 * (w * z_rh + x * y)
    local cosr_cosp = 1 - 2 * (y*y + z_rh*z_rh)
	-- 6) Roll  = atan2( 2*(w*z_rh + x*y),  1 - 2*(y^2 + z_rh^2) )
    local roll_raw  = math.atan2(sinr_cosp, cosr_cosp)

	if yaw > -math.pi/2 and yaw < math.pi/2 then
		roll_offset = 0
	else
		roll_offset = math.pi
	end
    local roll = WrapAngle(roll_raw - roll_offset)

    return {
        pitch = pitch,
        yaw   = yaw,
        roll  = roll,
    }
end
------------------------------------------------


-- placement facing ---------------------------
-- for body, +x = rightward; +z = backward; +y = upward
local yawMap = {
	north =   0,
	east  =  -math.pi/2,
	south = math.pi,
	west  = math.pi/2,
}

local function getRefQuat(face)
	local rad = yawMap[face] or 0
	local q = {
		w = math.cos(rad/2),
		x = 0,
		y = math.sin(rad/2),
		z = 0,
	}
	return q
end

local refQuat = getRefQuat(properties.placementFace) -- quaternion to reflect placement facing
------------------------------------------------


-- ship data (position, facing) ---------------
local shipData = {
	lastPos = nil,
	pos = nil,
	velocity = nil,
	omega = nil,
	rotation = nil,
	quat = nil,
	quat_w = nil,
	mass = 0,
	inertiaT = nil,
	lock = false, -- whether locking position
	level = false, -- whether auto-trimming
	prev_pitch = nil, -- record previous pitch for autotrimming
	prev_roll = nil, -- record previous pitch for autotrimming
	autopilot = false, -- whether autopilot
	autopilot_trim = true, -- whether trim pitch & roll in autopilot
	prev_yaw = nil, -- previous yaw in autopilot
	autopilot_yaw = true, -- whether adjust yaw in autopilot
	autopilot_message = nil, -- autopilot updates for display
	lastY = nil, -- record y for starship mode
	euler = nil, -- euler from physics_tick
	id = ship.getId(), -- ID of the ship
	user = nil, -- name of the user
}

-- update ship data with physics_tick
shipData.update_phys_tick = function(phys)
	local poseVel = phys.getShipPoseVel()
	shipData.pos = poseVel.pos
	shipData.velocity = poseVel.velocity
	shipData.omega = poseVel.omega
	shipData.rotation = poseVel.rot
	-- get current adjusted quaternion based on intial facing
	shipData.quat_w = ship.getQuaternion()
	shipData.quat = QuatMultiply(refQuat, shipData.quat_w)
	local inertia = phys.getInertia()
	shipData.mass = inertia.mass
	shipData.inertiaT = inertia.momentOfInertiaTensor
	shipData.euler = phys.getControllerEuler()
end

-- update ship data with cc tick
shipData.update_cc_tick = function()
	shipData.pos = ship.getWorldspacePosition()
	shipData.velocity = ship.getVelocity()
	shipData.omega = ship.getOmega()
	-- shipData.rotation = poseVel.rot
	-- get current adjusted quaternion based on intial facing
	shipData.quat_w = ship.getQuaternion()
	shipData.quat = QuatMultiply(refQuat, shipData.quat_w)
	shipData.mass = ship.getMass()
	shipData.inertiaT = ship.getMomentOfInertiaTensor()
end
------------------------------------------------


-- PID control --------------------------------
local pidControl = {
	dt = 0.05,
	state = { -- record pid states for vx vy vz
		ex_prev = 0, -- previous error in vx
		ey_prev = 0, -- previous error in vy
		ez_prev = 0, -- previous error in vz
		Ix_acc = 0, -- integral error in vx
		Iy_acc = 0, -- integral error in vy
		Iz_acc = 0, -- integral error in vz
	}
}

-- PID control on vx vy vz
-- k = PID param of the ship; vmax.x, vmax.y, vmax.z = max V for xyz; locky = whether lock Y, shipData.lastY provided
pidControl.update_v3d = function (k, vmax, input_v_scale, locky)
	-- k = PID params for vx vy vz; vmax = {vx_max, vy_max, vz_max}; input = input from controller
	local v_des_w, v_des
	if shipData.lock then
		v_des_w = {
			x = Scale2Max((shipData.lastPos.x-shipData.pos.x)/pidControl.dt, vmax.x),
			y = Scale2Max((shipData.lastPos.y-shipData.pos.y)/pidControl.dt, vmax.y),
			z = Scale2Max((shipData.lastPos.z-shipData.pos.z)/pidControl.dt, vmax.z),
		}
		-- v_des = WorldToBody({x = v_des_w.x/10, y = v_des_w.y/10, z = v_des_w.z/10}, shipData.quat)
		v_des = WorldToBody(v_des_w, shipData.quat)
	else
		local norm_v_scale = NormVector(input_v_scale)
		v_des = { -- desired v based on user input and vmax
			x = norm_v_scale.x * vmax.x,
			y = norm_v_scale.y * vmax.y,
			z = norm_v_scale.z * vmax.z,
		}
		if locky then
			local v_locky_w = {
				x = 0,
				y = Scale2Max((shipData.lastY-shipData.pos.y)/pidControl.dt, vmax.y),
				z = 0,
			}
			local v_locky_des = WorldToBody(v_locky_w, shipData.quat)
			v_des = {
				x = v_des.x+v_locky_des.x,
				y = v_des.y+v_locky_des.y,
				z = v_des.z+v_locky_des.z,
			}
		end
	end

	local v_w = shipData.velocity -- ship velocity in world coordinate

	local v_body = WorldToBody(v_w, shipData.quat) -- transform v to body coordinate
	local e = { -- current error
		x = v_des.x - v_body.x,
		y = v_des.y - v_body.y,
		z = v_des.z - v_body.z,
	}

	-- update integral and calculate error derivative
	pidControl.state.Ix_acc = pidControl.state.Ix_acc + e.x * pidControl.dt
	pidControl.state.Iy_acc = pidControl.state.Iy_acc + e.y * pidControl.dt
	pidControl.state.Iz_acc = pidControl.state.Iz_acc + e.z * pidControl.dt

	local dexdt = (e.x - pidControl.state.ex_prev) / pidControl.dt
	local deydt = (e.y - pidControl.state.ey_prev) / pidControl.dt
	local dezdt = (e.z - pidControl.state.ez_prev) / pidControl.dt

	pidControl.state.ex_prev, pidControl.state.ey_prev, pidControl.state.ez_prev = e.x, e.y, e.z

	-- acceleration on body from PID
	local a_body = {
		x = k.x.P*e.x + k.x.I*pidControl.state.Ix_acc + k.x.D*dexdt,
		y = k.y.P*e.y + k.y.I*pidControl.state.Iy_acc + k.y.D*deydt,
		z = k.z.P*e.z + k.z.I*pidControl.state.Iz_acc + k.z.D*dezdt,
	}

	-- force on body
	local m = shipData.mass
	local f_body = { x=a_body.x*m, y=a_body.y*m, z=a_body.z*m }

	engine.applyRotDependentForce(f_body.x, f_body.y, f_body.z) -- based on local force
end

-- PID control on omega
-- k = PID parameters for omega
-- omega_max = maximum omega for x y z
-- input_omega_scale = input [-1,1],[-1,1],[-1,1] scaling for omega
-- level = boolean, whether level according to euler_des
-- euler_des = {pitch_des, yaw_des, roll_des}
pidControl.update_omega = function(k, omega_max, input_omega_scale, level_flag, euler_des)
	-- transform to body coordinate
	local omega_b = WorldToBody(shipData.omega, shipData.quat)

	-- whether auto trimming roll & pitch
	local tx_level = 0
	local ty_level = 0
	local tz_level = 0
	if shipData.level then
		local euler_cur = Quat2Euler_Vector(shipData.quat)

		if level_flag.pitch then
			local pitch_err = euler_des.pitch - euler_cur.pitch
			tx_level = pitch_err * k.level.P + (-omega_b.x)*k.level.D
			if math.abs(euler_cur.pitch) > math.pi/2 then
				tx_level = -1 * tx_level
			end
		end
		if level_flag.yaw then
			local yaw_err = euler_des.yaw - euler_cur.yaw
			ty_level = yaw_err * k.level.P + (-omega_b.y)*k.level.D
		end
		if level_flag.roll then
			local roll_err = euler_des.roll - euler_cur.roll
			tz_level = roll_err * k.level.P + (-omega_b.z)*k.level.D
		end
	end

	-- target omega
	local omega_des = {
		x = omega_max.x * input_omega_scale.x,
		y = omega_max.y * input_omega_scale.y,
		z = omega_max.z * input_omega_scale.z,
	}
	local omega_err = {
		x = omega_des.x - omega_b.x,
		y = omega_des.y - omega_b.y,
		z = omega_des.z - omega_b.z,
	}

	-- PD control
	-- omega control from input + trimming
	local tx = omega_err.x * k.omega.P + (-omega_b.x) * k.omega.D + tx_level
	local ty = omega_err.y * k.omega.P + (-omega_b.y) * k.omega.D + ty_level
	local tz =  omega_err.z * k.omega.P + (-omega_b.z) * k.omega.D - tz_level

	local I = shipData.inertiaT  -- 3×3 table

	-- τ = I · [tx,ty,tz]
	local tau = MulMatVec(shipData.inertiaT, {x=tx, y=ty, z=tz})
	engine.applyRotDependentTorque(tau.x, tau.y, tau.z)
end

pidControl.v_xz2Yaw = function (vx, vz, prop)
	local r = math.sqrt(vx*vx+vz*vz)
	if r < 0.01 then
		r = 0.01
	end
	if vx > 0 then
		return WrapAngle(-math.asin(vz/r)+prop.NESWYaw.E)
	else
		return WrapAngle(-(math.pi-math.asin(vz/r))+prop.NESWYaw.E)
	end
end

-- auto pilot to waypoint
pidControl.autoPilot = function (prop, wp)
	local target = wp.targets[wp.targetid] -- destination
	local y_diff = prop.cruiseY - shipData.pos.y
	local x_diff = target.x - shipData.pos.x
	local z_diff = target.z - shipData.pos.z
	local euler = Quat2Euler_Vector(shipData.quat)
	if math.abs(x_diff) > 0.5 or math.abs(z_diff) > 0.5 then -- has not reached target xz, set to cruiseY
		if math.abs(y_diff) > 0.5 then
			shipData.autopilot_message = {"reaching", "cruiseY ..."}
			shipData.lastPos = { -- set lock position to cruiseY
				x = shipData.pos.x,
				y = prop.cruiseY,
				z = shipData.pos.z,
			}
			shipData.lock = true
			pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
			pidControl.update_omega(prop.K, prop.Omega_max, {x=0,y=0,z=0}, {pitch=false, yaw=false, roll=false}, {pitch=nil, yaw=nil, roll=nil})
		else
			shipData.lock = false
			-- reach cruiseY, start autopilot
			-- adjust yaw to face the target
			local target_yaw = pidControl.v_xz2Yaw(x_diff, z_diff, prop)
			local pitch_diff = 0 - euler.pitch
			local roll_diff = 0 - euler.roll
			local yaw_diff = target_yaw - euler.yaw
			if shipData.prev_yaw == nil then -- initialize previous yaw diff
				shipData.prev_yaw = euler.yaw
			end

			if shipData.autopilot_trim and math.abs(roll_diff) > 0.01 or math.abs(pitch_diff) > 0.01 then
				shipData.autopilot_message = {"trimming", "pitch/roll ..."}
				shipData.level = true
				pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
				pidControl.update_omega(prop.K, prop.Omega_max, {x=0,y=0,z=0}, {pitch=true, yaw=false, roll=true}, {pitch=0, yaw=nil, roll=0})
			else
				shipData.autopilot_trim = false -- not trimming in the following autopilot
				if euler.yaw - shipData.prev_yaw < -math.pi then -- yaw was wrapped
					yaw_diff = euler.yaw + 2*math.pi - target_yaw
				end
				if euler.yaw - shipData.prev_yaw > math.pi then -- yaw was wrapped
					yaw_diff = euler.yaw - 2*math.pi - target_yaw
				end
				shipData.prev_yaw = euler.yaw
				if shipData.autopilot_yaw and math.abs(yaw_diff) > 0.05 then
					shipData.autopilot_message = {"adjusting yaw", "towards target ..."}
					shipData.level = true
					pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
					pidControl.update_omega(prop.K, prop.Omega_max, {x=0,y=0,z=0}, {pitch=false, yaw=true, roll=false}, {pitch=0, yaw=target_yaw, roll=0})
				else
					-- correct yaw, move forward
					shipData.autopilot_message = {"forward ..."}
					shipData.autopilot_yaw = false -- not adjusting yaw in the following autopilot
					shipData.lastPos = { -- set lock position to cruiseY
						x = target.x,
						y = prop.cruiseY,
						z = target.z,
					}
					shipData.lock = true
					pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
					shipData.level = false
					pidControl.update_omega(prop.K, prop.Omega_max, {x=0,y=0,z=0}, {pitch=false, yaw=false, roll=false}, {pitch=nil, yaw=nil, roll=nil})
				end
			end
		end
	else -- reach target x z
		shipData.level = true -- maintain target yaw all time
		pidControl.update_omega(prop.K, prop.Omega_max, {x=0,y=0,z=0}, {pitch=true, yaw=true, roll=true}, {pitch=0, yaw=target.yaw, roll=0})

		if math.abs(euler.yaw-target.yaw) > 0.01 then -- reach target adjust yaw
			shipData.autopilot_message = {"adjusting to", "target yaw ..."}
			shipData.lastPos = { -- adjust to target yaw at cruiseY
				x = target.x,
				y = prop.cruiseY,
				z = target.z,
			}
			shipData.lock = true
			pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
		else
			shipData.lastPos = { -- adjust to target y
					x = target.x,
					y = target.y,
					z = target.z,
				}
				pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
			if math.abs(target.y-shipData.pos.y) > 0.5 then -- reach target x z
				shipData.autopilot_message = {"adjusting to", "target Y ..."}
			else
				shipData.autopilot_message = {"Autopilot done!"}
			end
		end
	end
end

-- Spaceship mode master control
-- input = controller input; prop = properties of the ship; wp = waypoint; phys = physics_tick input
pidControl.spaceShip = function (input, prop, wp, phys)
	engine.applyInvariantForce(0, prop.gravity_mult[prop.tick_mode]*shipData.mass, 0)
	if phys == nil then
		shipData.update_cc_tick()
	else
		shipData.update_phys_tick(phys)
	end

	-- autopilot setting
	local currLeftJoyClick = input.LeftJoyClick
	if currLeftJoyClick and not prevLeftJoyClick then
		if shipData.autopilot then
			shipData.autopilot = false
			shipData.lock = false
			shipData.level = false
		else
			shipData.autopilot_trim = true -- do autotrimming when autopilot is turn on
			shipData.autopilot_yaw = true -- adjust yaw when autopilot is turn on
			shipData.autopilot = true
		end
	end
	prevLeftJoyClick = currLeftJoyClick

	-- autopilot override controller input
	if shipData.autopilot then
		pidControl.autoPilot(prop, wp)
	else
		-- manual
		-- lock position setting
		local currStart = input.start
		if currStart and not prevStart then
			if shipData.lock then
				shipData.lock = false
			else
				shipData.lastPos = {
					x = shipData.pos.x,
					y = shipData.pos.y,
					z = shipData.pos.z,
				}
				shipData.lock = true
			end
		end
		prevStart = currStart

		-- auto-trimming
		local currBack = input.back
		if currBack and not prevBack then
			if shipData.level then
				shipData.level = false
			else
				shipData.level = true
			end
		end
		prevBack = currBack

		-- implement control
		if shipData.lock then
			pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
			pidControl.update_omega(prop.K, prop.Omega_max, {x=0, y=0, z=0}, {pitch=false, yaw=false, roll=false})
		else
			-- for body, +x = rightward; +z = backward; +y = upward
			-- input = input from controller; prop = properties
			local input_v_scale = {
				x = (input.right == input.left and 0) or (input.right and 1 or -1),
				y = (input.up == input.down and 0) or (input.up and 1 or -1),
				z = input.LeftStick.y,
			}
			pidControl.update_v3d(prop.K, prop.Vmax, input_v_scale)

			-- for body, x = pitch; y = yaw; z = roll
			local input_omega_scale = {
				x = input.RightStick.y,
				y = -input.LeftStick.x,
				z = (input.LB == input.RB and 0) or (input.LB and 1 or -1),
			}
			pidControl.update_omega(prop.K, prop.Omega_max, input_omega_scale, {pitch=true, yaw=false, roll=true}, {pitch=0, yaw=nil, roll=0})
		end
	end
end

-- input = controller input; prop = properties of the ship; wp = waypoint; phys = physics_tick input
pidControl.starShip = function (input, prop, wp, phys)
	engine.applyInvariantForce(0, prop.gravity_mult[prop.tick_mode]*shipData.mass, 0)
	if phys == nil then
		shipData.update_cc_tick()
	else
		shipData.update_phys_tick(phys)
	end

	-- initialize Y recording
	if shipData.lastY == nil then
		shipData.lastY = shipData.pos.y
	end

	-- autopilot setting
	local currLeftJoyClick = input.LeftJoyClick
	if currLeftJoyClick and not prevLeftJoyClick then
		if shipData.autopilot then
			shipData.autopilot = false
			shipData.lock = false
			shipData.level = false
		else
			shipData.autopilot_trim = true -- do autotrimming when autopilot is turn on
			shipData.autopilot_yaw = true -- adjust yaw when autopilot is turn on
			shipData.autopilot = true
		end
	end
	prevLeftJoyClick = currLeftJoyClick

	-- autopilot override controller input
	if shipData.autopilot then
		pidControl.autoPilot(prop, wp)
	else
		-- manual
		-- lock position setting
		local currStart = input.start
		if currStart and not prevStart then
			if shipData.lock then
				shipData.lock = false
			else
				shipData.lastPos = {
					x = shipData.pos.x,
					y = shipData.pos.y,
					z = shipData.pos.z,
				}
				shipData.lock = true
			end
		end
		prevStart = currStart

		-- implement control
		if shipData.lock then
			pidControl.update_v3d(prop.K, prop.Vmax, {x=0, y=0, z=0})
			pidControl.update_omega(prop.K, prop.Omega_max, {x=0, y=0, z=0}, {pitch=false, yaw=false, roll=false})
		else
			-- for body, +x = rightward; +z = backward; +y = upward
			-- input = input from controller; prop = properties
			local input_v_scale = {
				x = (input.right == input.left and 0) or (input.right and 1 or -1),
				y = (input.up == input.down and 0) or (input.up and 1 or -1),
				z = input.LeftStick.y,
			}
			if input_v_scale.y == 0 then -- no input for altitude adjustment
				pidControl.update_v3d(prop.K, prop.Vmax, input_v_scale, true)
			else
				pidControl.update_v3d(prop.K, prop.Vmax, input_v_scale)
				shipData.lastY = shipData.pos.y -- update current y
			end

			-- for body, x = pitch; y = yaw; z = roll
			local input_omega_scale = {
				x = 0,
				y = -input.LeftStick.x,
				z = 0,
			}
			shipData.level = true
			pidControl.update_omega(prop.K, prop.Omega_max, input_omega_scale, {pitch=true, yaw=false, roll=true}, {pitch=0, yaw=nil, roll=0})
		end
	end
end
------------------------------------------------


--HUD Stuffs--------------------------------------------------------------------
local my_4x5_number = {
    {0,1,1,0, 1,0,0,1, 1,0,0,1, 1,0,1,1, 0},  -- “0”
    {0,1,1,0, 0,0,1,0, 0,0,1,0, 0,0,1,1, 1},  -- “1”
    {0,1,1,0, 1,0,0,1, 0,0,1,0, 0,1,1,1, 1},  -- “2”
    {1,1,1,1, 0,0,0,1, 0,1,1,1, 0,0,0,1, 1},  -- “3”
    {1,0,0,1, 1,0,0,1, 1,1,1,1, 0,0,0,1, 1},  -- “4”
    {1,1,1,1, 1,0,0,0, 1,1,1,1, 0,0,0,1, 1},  -- “5”
    {1,1,1,1, 1,0,0,0, 1,1,1,1, 1,0,0,1, 1},  -- “6”
    {1,1,1,1, 0,0,0,1, 0,0,1,0, 0,1,0,0, 0},  -- “7”
    {1,1,1,1, 1,0,0,1, 1,1,1,1, 1,0,0,1, 1},  -- “8”
    {1,1,1,1, 1,0,0,1, 1,1,1,1, 0,0,0,1, 1},  -- “9”
}

-- 5x5 letters and digits
local my_5x5_letter = {
	["_cross"] ={ 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1,},
	["_cross_box"] = { 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1,},
	["_"] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,},
	["-"] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
	["/"] = { 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,},
	["<"] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0,},
	[">"] = { 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,},
	["a"] = { 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1,},
	["b"] = { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0,},
	["c"] = { 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,},
	["d"] = { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0,},
	["e"] = { 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,},
	["f"] = { 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,},
	["g"] = { 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1,},
	["h"] = { 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1,},
	["i"] = { 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1,},
	["j"] = { 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0,},
	["k"] = { 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1,},
	["l"] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,},
	["m"] = { 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1,},
	["n"] = { 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1,},
	["o"] = { 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0,},
	["p"] = { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,},
	["q"] = { 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1,},
	["r"] = { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1,},
	["s"] = { 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0,},
	["t"] = { 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,},
	["u"] = { 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0},
	["v"] = { 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0,},
	["w"] = { 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0,},
	["x"] = { 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1,},
	["y"] = { 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,},
	["z"] = { 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1,},
	["0"] = { 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0,},
	["1"] = { 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,},
	["2"] = { 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1,},
	["3"] = { 0, 3, 3, 3, 0, 0, 0, 0, 0, 3, 0, 0, 3, 3, 3, 0, 0, 0, 0, 3, 0, 3, 3, 3, 0,},
	["4"] = { 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,},
	["5"] = { 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, },
	["6"] = { 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, },
	["7"] = { 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, },
	["8"] = { 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, },
	["9"] = { 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, },
	}

-- 把上面的 0/1 阵列转换成实际可以 Blit 的“ARGB 缓冲区”，
-- 0 表示透明，1 表示用示例颜色（白或半透明白）
local function BakeBitMap(buf, color)
    local b = {}
    for i = 1, #buf do
        if buf[i] == 1 then
            b[i-1] = color
        else
            b[i-1] = 0x00000000
        end
    end
    return b
end

-- 生成彩色和白色两个版本的数字位图
local my_4x5_number_white = {}
local my_4x5_number_gray  = {}
for d = 1, #my_4x5_number do
    my_4x5_number_gray[d]  = BakeBitMap(my_4x5_number[d], 0x33FFFFFF)  -- 半透明白
    my_4x5_number_white[d] = BakeBitMap(my_4x5_number[d], 0xFFFFFFFF)  -- 实心白
end

-- transparent_white/white/red
local my_5x5_letter_white = {}
local my_5x5_letter_red   = {}
for k, v in pairs(my_5x5_letter) do
    my_5x5_letter[k]     = BakeBitMap(v, 0x33FFFFFF)  -- 半透明白
    my_5x5_letter_white[k] = BakeBitMap(v, 0xFFFFFFFF)  -- 纯白
    my_5x5_letter_red[k]   = BakeBitMap(v, 0xFF1111FF)  -- 红色
end

--------------------------------------------------------------------------------
-- HUD 模块：放在文件中合适的位置（原来你定义 HUD 的地方）
--------------------------------------------------------------------------------
local HUD = {}
HUD.__index = HUD

-- 2D 向量构造
local function new2dVec(x, y)
    return { x = x or 0, y = y or 0 }
end

-- 3D 向量构造
local function newVec(x, y, z)
    if type(x) == "table" then
        return { x = x.x, y = x.y, z = x.z }
    else
        return { x = x or 0, y = y or 0, z = z or 0 }
    end
end

-- 2×2 矩阵 × 2D 向量
local function mat2_mul(m, v)
    return {
        x = m[1][1] * v.x + m[1][2] * v.y,
        y = m[2][1] * v.x + m[2][2] * v.y
    }
end

--------------------------------------------------------------------------------
-- 构造函数：接受一个已经 wrap 好的 hologram 周边对象
--------------------------------------------------------------------------------
function HUD.new(holo_screen)
    local self = setmetatable({}, HUD)
    self.screen = holo_screen

    -- HUD 在 Hologram 上的分辨率写死 384×256
    self.width  = 384
    self.height = 256

    -- 摄像机（“眼睛”）在飞船坐标系中的偏移（米为单位）
    self.eye_offset = newVec(2, hologram_eye_offset, 0)

    -- HUD 整体缩放（推荐取 4、5、6 等，看画面效果）
    self.scale = 5

    -- 给 HUD 四周留一个 0 的边框（方便调试，之后可以恢复成 0.1*width）
    -- 如果设成 0，则任何在屏幕边界外的线/数字都不会被剪裁
    self.attBorder = new2dVec(0, 0)
    -- 如果你想恢复 “留 10% 边框”，可以改成下面这一行：
    -- self.attBorder = new2dVec(0.1 * self.width, 0.1 * self.height)

    -- HUD 的中心点（像素坐标系）
    self.midPoint = new2dVec(self.width / 2, self.height / 2)

    -- 计算“眼睛偏移长度”，用于后续把俯仰误差换到像素上
    local eoff = self.eye_offset
    self.eye_len = math.sqrt(eoff.x*eoff.x + eoff.y*eoff.y + eoff.z*eoff.z)
    -- 这个 eye_pitch_offset 是让 HUD 在 y 方向略微上移一点，看起来更自然
    self.eye_pitch_offset =
        math.tan(math.asin(eoff.y / self.eye_len))   -- （弧度）
        * eoff.x / 2 / self.scale * self.midPoint.y  -- 转换到“像素值”

    -- 设置 Hologram 的基础属性
    self.screen.SetClearColor(0x00000000)       -- 背景透明
    self.screen.Resize(self.width, self.height) -- 分辨率 384×256
    local sc = (16 / self.width) * self.scale    -- 把“HUD 单位坐标”转换成像素
    self.screen.SetScale(sc, sc)
    -- 把整个坐标系往上平移一点，以模拟“眼睛高度”
    self.screen.SetTranslation(0, self.eye_offset.y, 0)
    -- 这里不做整体旋转，后续每条线会自己绕中心点做 Roll
    self.screen.SetRotation(0, 0, 0)

    return self
end

--------------------------------------------------------------------------------
-- 辅助：检测一个 2D 点是否处于“可绘制区域”（未超出 attBorder）
--------------------------------------------------------------------------------
function HUD:checkInBounds(p2d)
    return p2d.x >= self.attBorder.x
       and p2d.x <= (self.width - self.attBorder.x)
       and p2d.y >= self.attBorder.y
       and p2d.y <= (self.height - self.attBorder.y)
end

--------------------------------------------------------------------------------
-- 辅助：绘制一段折线
--   point  = {x=...,y=...}，表示“以屏幕中心为基准的偏移（像素）”
--   list   = { {x,y}, {x,y}, ... }，是一组相对单位坐标（会乘以 midPoint.x → 像素）
--   color  = 16 进制颜色
--------------------------------------------------------------------------------
function HUD:drawLineGroup(point, list, color)
    for i = 1, #list - 1 do
        local p1 = {
            x = point.x + list[i].x * self.midPoint.x,
            y = point.y + list[i].y * self.midPoint.x
        }
        local p2 = {
            x = point.x + list[i+1].x * self.midPoint.x,
            y = point.y + list[i+1].y * self.midPoint.x
        }
        if self:checkInBounds(p1) and self:checkInBounds(p2) then
            self.screen.DrawLine(p1.x, p1.y, p2.x, p2.y, color, 1)
        end
    end
end

--------------------------------------------------------------------------------
-- 辅助：在屏幕上绘制一个整数 n（可以是正负），字体大小 4×5 像素
--   pos        = {x=...,y=...}，屏幕像素坐标  
--   n          = 要绘制的整数  
--   forceWhite = true/false，true 用实心白，false 用半透明灰  
--------------------------------------------------------------------------------
function HUD:drawNumber(pos, n, forceWhite)
    if type(n) ~= "number" then return end
    n = math.floor(n + 0.5)
    local s = tostring(math.abs(n))
    local x0 = pos.x - 2 * #s    -- 4 像素宽度 + 1 像素间隔，总共 5 * #s / 2
    local y0 = pos.y - 2         -- 垂直上移 2 像素，让数字位于刻度线旁边

    for i = 1, #s do
        local digit = tonumber(s:sub(i, i))
        local bitmap = forceWhite
                     and my_4x5_number_white[digit + 1]
                     or my_4x5_number_gray[digit + 1]
        -- Bitmap 的宽度是 4，高度是 5
        self.screen.Blit(
            x0, y0,             -- 目标左上角像素坐标
            4, 5,               -- 宽 4、高 5
            bitmap,             -- 4×5 的像素颜色数组
            1                   -- 缩放因子 1（像素对像素）
        )
        x0 = x0 + 5  -- 下一个数字在右边 5 像素的位置
    end
end

--------------------------------------------------------------------------------
-- 核心：根据传入的 pitch/roll/yaw（**弧度**），在 HUD 上绘制地平线与刻度
--------------------------------------------------------------------------------
-- 注意：这里传入的 pitch, roll, yaw 都是弧度，不要再对它们做 math.rad
function HUD:drawHUD(pitch, roll, yaw)
    -- 1) 清屏
    self.screen.Clear()

    -- 2) 先根据俯仰算出“眼睛在世界坐标里 Y 方向的偏移”  
    local eho = newVec(self.eye_offset.x, 0, self.eye_offset.z)
    eho.y = math.sin(pitch) * (self.eye_offset.y)

    -- 3) 计算“俯仰误差”：o_ag = pitch - asin(eho.y / 眼睛长度)
    local eye_err = math.asin(eho.y / math.sqrt(eho.x*eho.x + eho.y*eho.y + eho.z*eho.z))
    local o_ag = pitch - eye_err

	local eye_len = math.sqrt(self.eye_offset.x^2 + self.eye_offset.y^2 + self.eye_offset.z^2)
    local tmp_err = math.asin( (math.sin(pitch) * self.eye_offset.y) / eye_len )  -- 俯仰误差
    local eye_pitch_offset_now =
    math.tan(tmp_err) * self.eye_offset.x / 2 / self.scale * self.midPoint.y

    -- 4) 根据 o_ag 计算“相对 HUD 的 Y 像素偏移”  
    --    offsetY = (eye_offset.x * tan(o_ag)) * 2 / self.scale
    local centerOffsetY = (eho.x * math.tan(o_ag)) * 2 / self.scale

    --------------------------------------------------------------------------------
    -- 5) 绘制中线（地平线），并且要按 Roll “反向”旋转
    --------------------------------------------------------------------------------
    if math.abs(centerOffsetY) < 0.5 - (self.attBorder.y / self.midPoint.y) then
        local line_center_raw = {
            new2dVec(0.1, 0),
            new2dVec(0.3, 0),
        }
        -- 在 Y 方向平移
        local moved = {}
        for i, v in ipairs(line_center_raw) do
            moved[i] = new2dVec(v.x, v.y + centerOffsetY)
        end

        -- 这里 Roll 取负号，使得飞船“往右 Roll”，地平线在屏幕上“往左倾”
        local sinR = -math.sin(roll)
        local cosR =  math.cos(roll)
        local rot2d = {{ cosR, -sinR }, { sinR, cosR }}

        local transformed = {}
        for i, v in ipairs(moved) do
            local tmp = mat2_mul(rot2d, v)
            transformed[i] = new2dVec(tmp.x, tmp.y)
        end

        -- 用纯白色画这条中线
        self:drawLineGroup(self.midPoint, transformed, 0xFFFFFFFF)
    end

    --------------------------------------------------------------------------------
    -- 6) 绘制上下刻度线（每隔 5° 一条，最远到 ±90°），以及数字标签
    --------------------------------------------------------------------------------
    local lint = math.rad(5)  -- 5° → 弧度
    for i = lint, math.rad(90), lint do
        -- “上方刻度”：o_ag + i
        local offsetY1 = (eho.x * math.tan(o_ag + i)) * 2 / self.scale
        if math.abs(offsetY1) < 0.5 - (self.attBorder.y / self.midPoint.y) then
            local raw = {
                new2dVec(0.1,    0),
                new2dVec(0.15,   0),
                new2dVec(0.15,   0.015),
            }
            local moved = {}
            for k, v in ipairs(raw) do
                moved[k] = new2dVec(v.x, v.y + offsetY1)
            end

            -- 同样 Roll 取负号
            local sinR = -math.sin(roll)
            local cosR =  math.cos(roll)
            local rot2d = {{ cosR, -sinR }, { sinR, cosR }}

            local transformed = {}
            for k, v in ipairs(moved) do
                local tmp = mat2_mul(rot2d, v)
                transformed[k] = new2dVec(tmp.x, tmp.y)
            end

            -- 半透明白色画刻度线
            self:drawLineGroup(self.midPoint, transformed, 0x33FFFFFF)

            -- 绘制刻度数字：角度 = -deg(i)
            local deg_i = -math.deg(i)
            -- 文本中心位置：先把 (-0.15, offsetY1) 这个点旋转得到屏幕相对中心的偏移
            local textOffset = new2dVec(-0.15, offsetY1)
            local p2 = mat2_mul(rot2d, textOffset)
            local numPos = new2dVec(
                self.midPoint.x + p2.x * self.midPoint.x,
                self.midPoint.y + p2.y * self.midPoint.x + eye_pitch_offset_now
            )
            -- 用白色数字（forceWhite = true）
            self:drawNumber(numPos, deg_i, true)
        end

        -- “下方刻度”：o_ag - i
        local offsetY2 = (eho.x * math.tan(o_ag - i)) * 2 / self.scale
        if math.abs(offsetY2) < 0.5 - (self.attBorder.y / self.midPoint.y) then
            local raw2 = {
                new2dVec(0.1,    0),
                new2dVec(0.15,   0),
                new2dVec(0.15,  -0.015),
            }
            local moved2 = {}
            for k, v in ipairs(raw2) do
                moved2[k] = new2dVec(v.x, v.y + offsetY2)
            end

            -- Roll 取负号
            local sinR = -math.sin(roll)
            local cosR =  math.cos(roll)
            local rot2d = {{ cosR, -sinR }, { sinR, cosR }}

            local transformed2 = {}
            for k, v in ipairs(moved2) do
                local tmp = mat2_mul(rot2d, v)
                transformed2[k] = new2dVec(tmp.x, tmp.y)
            end

            self:drawLineGroup(self.midPoint, transformed2, 0x33FFFFFF)

            -- 绘制刻度数字：角度 = +deg(i)
            local deg_i = math.deg(i)
            local textOffset2 = new2dVec(-0.15, offsetY2)
            local p22 = mat2_mul(rot2d, textOffset2)
            local numPos2 = new2dVec(
                self.midPoint.x + p22.x * self.midPoint.x,
                self.midPoint.y + p22.y * self.midPoint.x + eye_pitch_offset_now
            )
            self:drawNumber(numPos2, deg_i, false)  -- 用半透明灰色
        end
    end

    -- 7) 最后刷新到 Hologram
    self.screen.Flush()
end

function DisplayCoord(monitor, pos, cursor)
	local text_table = {"X = ", "Y = ", "Z = "}
	for i = 0,2 do
		monitor.setCursorPos(1, cursor+i)
		monitor.write(text_table[i+1])
		monitor.write(math.modf(pos[i+1]))
	end
end

-- display flight data
function DisplayFlightData(monitor, name, id, user, rpm, tick_mode, mode, pos, vel, euler, lock, level, auto, cursor)
	monitor.clear()
	-- write name
	monitor.setTextColor(Color_highlight)
	monitor.setBackgroundColor(Color_bg_highlight)
	monitor.setCursorPos(1, cursor)
	monitor.write(name)
	monitor.setCursorPos(1, cursor+1)
	monitor.setTextColor(Color_default)
	monitor.setBackgroundColor(Color_bg_default)
	monitor.write("ShipID: ")
	monitor.write(id)
	monitor.setCursorPos(1, cursor+2)
	monitor.write("User: ")
	monitor.write(user)
	monitor.setCursorPos(1, cursor+3)
	monitor.write("RPM: ")
	monitor.write(math.ceil(rpm))

	-- display tick mode
	cursor = cursor + 4
	monitor.setTextColor(Color_highlight)
	monitor.setBackgroundColor(Color_bg_highlight)
	monitor.setCursorPos(1, cursor)
	if tick_mode == 1 then
		monitor.write("CC tick")
	elseif tick_mode == 2 then
		monitor.write("Physics tick")
	end

	-- display shipmode
	cursor = cursor + 2
	monitor.setCursorPos(1, cursor)
	monitor.setTextColor(Color_highlight)
	monitor.setBackgroundColor(Color_bg_highlight)
	monitor.write(properties.mode_name[mode])

	-- display coordinate
	cursor = cursor + 2
	monitor.setTextColor(Color_default)
	monitor.setBackgroundColor(Color_bg_default)
	if auto.on then
		monitor.setTextColor(Color_auto)
	elseif lock then
		monitor.setTextColor(Color_lock)
	else
		monitor.setTextColor(Color_default)
	end
	DisplayCoord(monitor, {pos.x, pos.y, pos.z}, cursor) -- display current position

	cursor = cursor + 4
	monitor.setCursorPos(1, cursor) -- display current velocity
	monitor.write("v = ")
	monitor.write(math.modf(vel*100)/100)
	monitor.write(" m/s")

	if auto.on then
		monitor.setTextColor(Color_auto)
	elseif lock then
		monitor.setTextColor(Color_lock)
	elseif level then
		monitor.setTextColor(Color_level)
	else
		monitor.setTextColor(Color_default)
	end

	cursor = cursor + 2
	monitor.setCursorPos(1, cursor) -- display current euler
	monitor.write("pitch: ")
	monitor.write(math.modf(euler.pitch/math.pi*180))
	monitor.setCursorPos(1, cursor+1)
	monitor.write("roll: ")
	monitor.write(math.modf(euler.roll/math.pi*180))
	monitor.setCursorPos(1, cursor+2)
	monitor.write("yaw: ")
	monitor.write(math.modf(euler.yaw/math.pi*180))

	cursor = cursor + 4
	if auto.on then
		monitor.setCursorPos(1, cursor)
		monitor.setTextColor(Color_highlight)
		monitor.setBackgroundColor(Color_bg_highlight)
		monitor.write("Autopilot")
		monitor.setTextColor(Color_auto)
		monitor.setBackgroundColor(Color_bg_default)
		for i, line in ipairs(auto.message) do
			cursor = cursor + 1
			monitor.setCursorPos(1, cursor)
			monitor.write(line)
		end
	end
end

-- display waypoint
function DisplayWaypoint(monitor, wp, cursor)
	monitor.clear()
	monitor.setCursorPos(1, cursor)
	for i, target in ipairs(wp.targets) do
		if i == wp.targetid then
			monitor.setTextColor(Color_highlight) -- highlight current target
			monitor.setBackgroundColor(Color_bg_highlight)
		else
			monitor.setTextColor(Color_default)
			monitor.setBackgroundColor(Color_bg_default)
		end
		monitor.setCursorPos(1, cursor)
		monitor.write("Destination ")
		monitor.write(i)
		monitor.setTextColor(Color_default)
		monitor.setBackgroundColor(Color_bg_default)
		monitor.setCursorPos(1, cursor+1)
		monitor.write(target.comment)
		DisplayCoord(monitor, {target.x, target.y, target.z}, cursor+2)
		monitor.setCursorPos(1, cursor+5)
		monitor.write("Yaw = ")
		monitor.write(math.modf(target.yaw/math.pi*180))
		cursor = cursor + 7
	end
end
--------------------------------------------------------------------------------


-- main program

-- initialize shipData
shipData.update_cc_tick()

-- initialize HUD
local holo = peripheral.find("hologram")
local hud = HUD.new(holo)

-- initialize monitor for waypoint
local monitor_waypoint = peripheral.wrap(Waypoint_side)
local waypoint_cursor = 2
local waypoint_lines = 6
monitor_waypoint.setTextScale(0.5)
monitor_waypoint.clear()

-- initialize monitor for flight data
local monitor_data = peripheral.wrap(Data_side)
local flightdata_cursor = 2
monitor_data.setTextScale(0.5)
monitor_data.clear()

-- initialize controller & authenticate identity
controller.getInput()
while true do
	local auth = false
	if controller.joy.hasUser() then
		for i, uuid in pairs(controller.AllowedUser) do
			if uuid == controller.joy.getUserUUID() then
				auth = true
				shipData.user = controller.AllowedUserName[i]
				break
			end
		end
		if auth then
			print("Authentication successful!")
			print("Flight control initiated")
			break
		else
			print("Authentication unsuccessful!")
		end
	end
	os.sleep(0.05)
end

-- flight control loop
while true do
	-- get input from controller
	local userInput = controller.getInput()

	if properties.tick_mode == 2 then -- use physics_tick
		pidControl.dt = 1/60
		local eventData = {os.pullEvent()}
		if eventData[1] == "monitor_touch" and eventData[2] == Waypoint_side then -- monitor_touch
			local idx = math.floor((eventData[4]-waypoint_cursor) / (waypoint_lines+1)) + 1
			if idx <= #waypoint.targets then
				if prev_targetid ~= idx then
					shipData.autopilot_yaw = true
					prev_targetid = idx
				end
				waypoint.targetid = idx
				waypoint.targetid = idx
			end
		elseif eventData[1] == "monitor_touch" and eventData[2] == Data_side then -- mode setting
			if eventData[4] == flightdata_cursor+6 then
				properties.mode = properties.mode + 1 -- switch to next drive mode
				if properties.mode > #properties.gravity_mult then
					properties.mode = 1 -- reset to 1
					shipData.level = false -- spaceship mode turn-off autotrimming
				end
			elseif eventData[4] == flightdata_cursor+4 then -- switch tick update mode
				properties.tick_mode = 3 - properties.tick_mode
			end
		elseif eventData[1] == "phys_tick" then -- physics_tick
			if properties.mode == 1 then
				pidControl.spaceShip(userInput, properties, waypoint, eventData[2])
			elseif properties.mode == 2 then
				pidControl.starShip(userInput, properties, waypoint, eventData[2])
			end

			-- draw HUD
			local euler = shipData.euler
			hud:drawHUD(euler.pitch, -euler.roll, euler.yaw)

			-- display info
			local vel = L2Norm(shipData.velocity)
			DisplayFlightData(monitor_data, properties.name, shipData.id, shipData.user, shipData.mass/mass_rpm_ratio, properties.tick_mode, properties.mode, shipData.pos, vel, euler, shipData.lock, shipData.level, {on=shipData.autopilot, message=shipData.autopilot_message}, flightdata_cursor)
			DisplayWaypoint(monitor_waypoint, waypoint, waypoint_cursor)
		end
	else
		-- monitor_touch
		local eventData = {os.pullEvent()}
		if eventData[1] == "monitor_touch" and eventData[2] == Waypoint_side then -- destination setting
			local idx = math.floor((eventData[4]-waypoint_cursor) / (waypoint_lines+1)) + 1
			if idx <= #waypoint.targets then
				if prev_targetid ~= idx then
					shipData.autopilot_yaw = true -- target changed, adjust yaw
					prev_targetid = idx
				end
				waypoint.targetid = idx
			end
		elseif eventData[1] == "monitor_touch" and eventData[2] == Data_side then -- mode setting
			if eventData[4] == flightdata_cursor+6 then
				properties.mode = properties.mode + 1 -- switch to next drive mode
				if properties.mode > #properties.mode_name then
					properties.mode = 1 -- reset to 1
					shipData.level = false -- spaceship mode turn-off autotrimming
				end
			elseif eventData[4] == flightdata_cursor+4 then  -- switch tick update mode
				properties.tick_mode = 3 - properties.tick_mode
			end
		end
		if properties.mode == 1 then
			pidControl.spaceShip(userInput, properties, waypoint, nil)
		elseif properties.mode == 2 then
			pidControl.starShip(userInput, properties, waypoint, nil)
		end

		-- draw HUD
		local euler = Quat2Euler_Vector(shipData.quat)
		hud:drawHUD(euler.pitch, euler.roll, euler.yaw)

		-- display info
		local vel = L2Norm(shipData.velocity)
		DisplayFlightData(monitor_data, properties.name, shipData.id, shipData.user, shipData.mass/mass_rpm_ratio, properties.tick_mode, properties.mode, shipData.pos, vel, euler, shipData.lock, shipData.level, {on=shipData.autopilot, message=shipData.autopilot_message}, flightdata_cursor)
		DisplayWaypoint(monitor_waypoint, waypoint, waypoint_cursor)

		os.sleep(pidControl.dt) -- update CC ticks 20 frames per second
	end
end
