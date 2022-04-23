include("MPC.jl");

m = mujoco_sim.get_model(joinpath(@__DIR__, "../descriptions/gen3.xml"))

function ctrler!(s) 
    u = zeros(7)
    setaction!(s, u)
end

d = mujoco_sim.get_data(m) 
sim = mujoco_sim.MJSim(m,d)   
mujoco_sim.simulate(sim, controller = ctrler!; mode="active")