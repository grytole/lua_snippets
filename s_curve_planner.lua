-- S-Curve Velocity Planner @ 28-jun-2021

-- S-curve velocity planner polynomial function
-- [in]  v0 : initial velocity for the phase (concave/linear/convex)
-- [in]  a0 : initial acceleration for the phase
-- [in]  jm : jerk value for the phase
-- [in]  t  : time from start of the phase
-- [out] vt : computed velocity for provided time value
-- [out] at : computed acceleration for provided time value
function compute(v0, a0, jm, t)
  local vt = v0 + (a0 * t) + ((jm * t * t) / 2.0)
  local at = a0 + (jm * t)
  return vt, at
end

-- S-curve trajectory generator
-- [in]  v0 : initial velocity for the trajectory
-- [in]  vs : target velocity for the trajectory
-- [in]  as : maximum acceleration for the trajectory
-- [in]  jm : constant jerk value for the trajectory
-- [in]  dt : time from start of trajectory
-- [out] completed : true - trajectory finished, false - othervise
-- [out] vt : computed velocity for provided time value
-- [out] at : computed acceleration for provided time value
function s_curve(v0, vs, as, jm, dt)
  local v0 = v0
  local vs = vs
  local as = as
  local jm = jm

  -- immediately finish trajectory without changes if incoming values are incorrect
  if (v0 == vs) or (0.0 <= as) or (0.0 <= jm) or (0.0 > dt) then
    return true, v0, 0.0
  end
  
  -- calculate velocity change during concave/convex phases
  local h_dv = (math.abs(vs - v0) / 2.0)
  local as_dv = ((as * as) / (2 * jm))
  local j_dv = math.min(as_dv, h_dv)

  -- direction coefficient
  local d_coeff = (v0 < vs) and (1.0) or (-1.0)

  -- velocities of concave/linear and linear/convex transitions
  local v1 = v0 + (d_coeff * j_dv)
  local v2 = vs - (d_coeff * j_dv)

  -- jerk values during phases
  local j01 = (d_coeff * jm)
  local j12 = 0.0
  local j2s = (d_coeff * (-jm))

  -- acceleration values for phase start points
  local a0 = 0.0
  local a1 = (d_coeff * math.sqrt(j_dv * 2 * math.abs(j01)))
  local a2 = a1

  -- period values of phases
  local p01 = math.abs(a1 / j01)
  local p12 = math.abs((v2 - v1) / a1)
  local p2s = p01

  -- timestamps of trajectory phase changes
  local t0 = 0.0
  local t1 = t0 + p01
  local t2 = t1 + p12
  local ts = t2 + p2s

  -- select current phase according to the incoming time value
  if dt >= t0 and dt <= t1 then
    -- concave phase calculations
    local vt, at = compute(v0, a0, j01, (dt - t0))
    return false, vt, at
  elseif dt > t1 and dt < t2 then
    -- linear phase calculations
    local vt, at = compute(v1, a1, j12, (dt - t1))
    return false, vt, at
  elseif dt >= t2 and dt <= ts then
    -- convex phase calculations
    local vt, at = compute(v2, a2, j2s, (dt - t2))
    return false, vt, at
  else
    --completed
    return true, vs, 0.0
  end
end


do
  -- incoming values for the velocity change action:
  --   change velocity from 500.0 points/quantum to 0.0 points/quantum with
  --   acceleration limit 10.0 points/quantum^2 and jerk value of 1.0 point/quantum^3
  -- FYI: move with direction change must be splitted on two trajectories at zero velocity point
  local v0 = 500.0
  local vs = 0.0
  local as = 10.0
  local jm = 1.0

  print(string.format("[in] v0:<%.02f> vs:<%.02f> as:<%.02f> jm:<%.02f>\n", v0, vs, as, jm))

  -- print out csv header
  print("dt,vt,at")

  -- time starts from zero for each trajectory
  -- system timestamp must be stored anywhere and be used as the start point
  local t = 0.0
  -- time step is 0.2 quantums
  local dt = 0.2

  repeat
    -- calculate single step
    local completed, vt, at = s_curve(v0, vs, as, jm, t)

    -- print out csv data
    print(string.format("%.2f,%.02f,%.02f", t, vt, at))

    -- jump to the next time value
    t = t + dt
  until completed

end
