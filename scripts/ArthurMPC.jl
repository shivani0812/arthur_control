include("MPC.jl")

qref = typeof(zeros(7))[]
push!(qref, [2.7099301624569527, -0.15201412303507578, 0.8178365155019467, -2.1885338731539914, 1.0147194304687517, 0.46298890832550743, 0.8702056834995249])
push!(qref, [2.728294974313176, -0.1720141230350758, 0.8003673335280456, -2.1781090510214, 1.0019535258622672, 0.472156233897044, 0.8743904054167106])
push!(qref, [2.781445516844634, -0.2298971473182204, 0.7498088793079459, -2.1479380393889884, 0.9650070675455945, 0.4986878603304811, 0.8865016234342443])
push!(qref, [2.8196404305509497, -0.27149290555995004, 0.7134766858045872, -2.1262566203319735, 0.938456693458179, 0.5177539532403241, 0.8952049574930507])
push!(qref, [2.8249935066192435, -0.2773226152353905, 0.7083846728458232, -2.123217936009418, 0.9347356175781703, 0.5204260955694391, 0.896424743185533])
# push!(qref, [2.824967309099577, -0.27756698510806643, 0.7078854910149355, -2.123396064969209, 0.9358763925095399, 0.5201436719781087, 0.896449825651394])
# push!(qref, [2.8408666049000453, -0.29756698510806645, 0.6897379479724371, -2.1090554351728628, 0.9221341571800112, 0.5241742346041602, 0.9017243389941612])
# push!(qref, [2.8800970774927217, -0.34691567713192406, 0.6449600723427398, -2.07367086901046, 0.8882260902301967, 0.5341193842899548, 0.9147388557205585])
# push!(qref, [2.894993506619243, -0.3656541537455642, 0.6279572067978956, -2.0602347912072747, 0.8753506624631211, 0.5378957144653587, 0.9196806729666474])

q̇ref = typeof(zeros(7))[]
push!(q̇ref, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
push!(q̇ref, [0.3672962371244665, -0.4, -0.34938363947802303, 0.20849644265182224, -0.25531809212969037, 0.183346511430731, 0.08369443834371632])
push!(q̇ref, [0.5655972556253911, -0.6159575824172965, -0.5380137547725984, 0.32106241189605733, -0.3931627869390016, 0.2823341848137957, 0.12888055975992246])
push!(q̇ref, [0.19830101850092452, -0.21595758241729635, -0.18863011529457524, 0.11256596924423506, -0.13784469480931116, 0.09898767338306463, 0.04518612141620611])
push!(q̇ref, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# push!(q̇ref, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# push!(q̇ref, [0.31798591600936743, -0.4, -0.36295086084996564, 0.28681259592692776, -0.2748447065905744, 0.08061125252103096, 0.10549026685534374])
# push!(q̇ref, [0.30779391355468544, -0.38717930330677697, -0.35131765359621164, 0.27761975267649003, -0.26603545503823534, 0.0780275214744486, 0.10210912006674494])
# push!(q̇ref, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

params = MPC_Params()

global not_nan = 0
global a_tot = [0,0,0]
for k=1:length(q̇ref)
    set_configuration!(params.state, qref[k])
    J = geometric_jacobian(params.state, path(params.model.mechanism, root_body(params.model.mechanism), bodies(params.model.mechanism)[end]))
    local vel_vec = Array(J)*q̇ref[k]
    a = vel_vec[4:6]/norm(vel_vec[4:6])
    if !isnan(a[1])
        global a_tot += a
        global not_nan += 1
    end
end
direction = a_tot / not_nan

Xref = typeof(zeros(14))[]
for k = 1:length(qref)
    push!(Xref, [qref[k]; q̇ref[k]])
end

Xref = interpolateTraj(params.model, Xref, params=params)

trajLength = length(Xref)

for k = 1:params.H
    push!(Xref, [qref[end]; q̇ref[end]])
end

prob = ArthurProblem(Xref, direction, params=params)
altro = ALTROSolver(prob, params.opts)
solve!(altro)
Z_track = TrajectoryOptimization.get_trajectory(prob)
global prob_mpc = ArthurHorizonProblem(prob, state(Z_track[1]), params.H, start=1)
global altro_mpc = ALTROSolver(prob_mpc, params.opts)
solve!(altro_mpc)

X_traj = typeof(zero(state(Z_track[1])))[]
push!(X_traj, state(Z_track[1]))
U_traj = typeof(zero(control(Z_track[1])))[]

B = 300000*Diagonal(ones(3))

J = getJ(params.model, prob_mpc.x0)
speed = typeof(zeros(3))[]
force = typeof(zeros(3))[]
print("Speed: ")
push!(speed, J * prob_mpc.x0[8:14])
println(speed[end])
print("Force: ")
push!(force, B * J * prob_mpc.x0[8:14])
println(force[end])

t0 = 0
global iter = 1
# max_iters = length(Z_track) + params.H
# println(max_iters)
# global k_mpc = 1

while norm(X_traj[end] - Xref[end]) > 0.01# && iter < max_iters
    # println(iter)
    global iter += 1
    global t0 += params.dt
    # Update the ALTRO solution, advancing forward by 1 time step
    push!(U_traj, control(prob_mpc.Z[1]))
    # mpc_update(altro_mpc, prob_mpc, Z_track, t0, k_mpc)
    x0 = rk4(prob_mpc.model, state(prob_mpc.Z[1]), control(prob_mpc.Z[1]), prob_mpc.Z[1].dt)
    # x0 = Xref[1]
    # mpc_update(altro_mpc, prob_mpc, Z_track, cons, x0, t0)
    k_mpc = min(trajLength, argmin(norm.([(states(Z_track)[k][1:7] - x0[1:7]) for k=1:length(Z_track)])))
    # if norm(Xref[k_mpc] - x0) < 1e-2
    #     global k_mpc = min(k_mpc+1, length(Xref))
    # end
    println(k_mpc)
    global prob_mpc = ArthurHorizonProblem(prob, x0, params.H, start=k_mpc)
    global altro_mpc = ALTROSolver(prob_mpc, params.opts)
    solve!(altro_mpc)
    
    push!(X_traj, prob_mpc.x0)
    J = getJ(params.model, prob_mpc.x0)
    print("Pos: ")
    print(norm(X_traj[end] - Xref[k_mpc]))
    print(" ")
    println(X_traj[end] - Xref[k_mpc])
    print("Speed: ")
    push!(speed, J * prob_mpc.x0[8:14])
    println(speed[end])
    print("Force: ")
    push!(force, B * J * prob_mpc.x0[8:14])
    println(force[end])
    # push!(errors, norm(X_traj[end] - Xref[end]))
    # solve!(altro_mpc)
end
# for traj in X_traj
#     println(traj)
# end
# for traj in U_traj
#     println(traj)
# end

X_sim = typeof(zero(state(Z_track[1])))[]
push!(X_sim, X_traj[1])
for k = 1:length(X_traj)-1
    # X_sim[k+1] = discrete_dynamics(TrajectoryOptimization.integration(prob_mpc), params.model, X_sim[k], U_traj[k], 0.0, params.dt)
    push!(X_sim, rk4(params.model, X_sim[k], U_traj[k], params.dt))
end
for traj in X_sim
    println(traj)
end
println(argmin(norm.([(X_sim[k] - Xref[end]) for k = 1:length(X_sim)])))
println(X_sim[argmin(norm.([(X_sim[k] - Xref[end]) for k = 1:length(X_sim)]))])
println(X_sim[argmin(norm.([(X_sim[k] - Xref[end]) for k = 1:length(X_sim)]))] - Xref[end])
println(norm(X_sim[argmin(norm.([(X_sim[k] - Xref[end]) for k = 1:length(X_sim)]))] - Xref[end]))

using MeshCat, MeshCatMechanisms, Blink
urdf = joinpath(@__DIR__,"../descriptions/gen3.urdf")
body = findbody(params.model.mechanism, "end_effector_link")
point = Point3D(default_frame(body), 0., 0, 0)
# Create the visualizer
vis = MechanismVisualizer(params.model.mechanism, URDFVisuals(urdf))

set_configuration!(params.state, qref[1])
p1 = transform(params.state, point, root_frame(params.model.mechanism))
set_configuration!(params.state, qref[end])
p2 = transform(params.state, point, root_frame(params.model.mechanism))
println(p2-p1)
println(norm(p2-p1))

# Render our target point attached to the robot as a sphere with radius 0.07
setelement!(vis, point, 0.05)

qs = Vector{Float64}[]
for x in X_traj
    push!(qs, copy(x[1:7]))
end
ts = collect(0:params.dt:(length(qs)-1)*params.dt);

setanimation!(vis, Animation(vis, ts, qs))
render(vis)

saveData = false
if saveData
    using JLD2
    println("Saving data")
    save_object(joinpath(@__DIR__,"../saved_data/x_reference.jld2"), Xref)
    println("Saved Xref")
    save_object(joinpath(@__DIR__,"../saved_data/x_traj.jld2"), X_traj)
    println("Saved X_traj")
    save_object(joinpath(@__DIR__,"../saved_data/x_sim.jld2"), X_sim)
    println("Saved X_sim")
    save_object(joinpath(@__DIR__,"../saved_data/u_traj.jld2"), U_traj)
    println("Saved U_traj")
    save_object(joinpath(@__DIR__,"../saved_data/cartesian_speed.jld2"), speed)
    println("Saved cartesian_speed")
    save_object(joinpath(@__DIR__,"../saved_data/force.jld2"), force)
    println("Saved force")
end