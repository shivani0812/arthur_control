include("MPC.jl");
using Plots
using JLD2
using Measures

params = MPC_Params()

println("Loading Data")
Xref = load_object(joinpath(@__DIR__,"../saved_data/x_reference.jld2"))
println("Loaded Xref")
Xref_short = load_object(joinpath(@__DIR__,"../saved_data/x_reference_short.jld2"))
println("Loaded Xref_short")
Xtraj = load_object(joinpath(@__DIR__,"../saved_data/x_traj.jld2"))
println("Loaded x_traj")
Xsim = load_object(joinpath(@__DIR__,"../saved_data/x_sim.jld2"))
println("Loaded x_sim")
Utraj = load_object(joinpath(@__DIR__,"../saved_data/u_traj.jld2"))
println("Loaded u_traj")
speed = load_object(joinpath(@__DIR__,"../saved_data/cartesian_speed.jld2"))
println("Loaded cartesian_speed")
force = load_object(joinpath(@__DIR__,"../saved_data/force.jld2"))
println("Loaded force")

urdf = joinpath(@__DIR__,"../descriptions/gen3.urdf")
body = findbody(params.model.mechanism, "end_effector_link")
point = Point3D(default_frame(body), 0., 0, 0)
set_configuration!(params.state, Xtraj[1][1:7])
p1 = transform(params.state, point, root_frame(params.model.mechanism))
distTraj = zeros(length(Xtraj))
trajX = zeros(length(Xtraj))
trajY = zeros(length(Xtraj))
trajZ = zeros(length(Xtraj))
eeTraj = typeof(zeros(3))[]
jointVelsTraj = zeros(length(Xtraj), 7)
for k = 1:length(Xtraj)
    set_configuration!(params.state, Xtraj[k][1:7])
    p2 = transform(params.state, point, root_frame(params.model.mechanism))
    distTraj[k] = 1000*norm(p2-p1)
    trajX[k] = p2.v[1]
    trajY[k] = p2.v[2]
    trajZ[k] = p2.v[3]
    push!(eeTraj, [p2.v[1], p2.v[2], p2.v[3]])
    jointVelsTraj[k, :] = Xtraj[k][8:14]
end

set_configuration!(params.state, Xref[1][1:7])
p1 = transform(params.state, point, root_frame(params.model.mechanism))
distRef = zeros(length(Xref))
refX = zeros(length(Xref))
refY = zeros(length(Xref))
refZ = zeros(length(Xref))
refSpeed = typeof(zeros(3))[]
eeRef = typeof(zeros(3))[]
jointVelsRef = zeros(length(Xref), 7)
for k = 1:length(Xref)
    J = getJ(params.model, Xref[k])
    push!(refSpeed, J * Xref[k][8:14])
    set_configuration!(params.state, Xref[k][1:7])
    J = getJ(params.model, prob_mpc.x0)
    p2 = transform(params.state, point, root_frame(params.model.mechanism))
    distRef[k] = 1000*norm(p2-p1)
    refX[k] = p2.v[1]
    refY[k] = p2.v[2]
    refZ[k] = p2.v[3]
    push!(eeRef, [p2.v[1], p2.v[2], p2.v[3]])
    jointVelsRef[k, :] = Xref[k][8:14]
end

trajError = zeros(length(Xtraj))
for k = 1:length(Xtraj)
    k_closest = argmin(norm.([(eeRef[i] - eeTraj[k]) for i=1:length(eeRef)]))
    trajError[k] = norm(eeTraj[k] - eeRef[k_closest])
end
println(trajError[end])

B = 300000*Diagonal(ones(3))
F_lim = 300 * direction
neg_rows = []
for i in 1:3
    if F_lim[i] < 0
        push!(neg_rows, i)
        F_lim[i] = F_lim[i] * -1
    end
end

jointTorquesTraj = zeros(length(Utraj), 7)
jointTorquesFromForce = zeros(length(Utraj), 7)
for k = 1:length(Utraj)
    jointTorquesTraj[k, :] = Utraj[k]
    J = getJ(params.model, Xref[k])
    jointTorquesFromForce[k, :] = J' * force[k]
end

refForce = zeros(length(Xref))
refXForce = zeros(length(Xref))
refYForce = zeros(length(Xref))
refZForce = zeros(length(Xref))
for k in 1:length(Xref)
    J = getJ(params.model, Xref[k])
    # for neg_row in neg_rows
    #     J[neg_row,:] = J[neg_row,:] * -1
    # end
    A_force = B * J
    f = A_force*Xref[k][8:14]
    refForce[k] = norm(f)
    refXForce[k] = f[1]
    refYForce[k] = f[2]
    refZForce[k] = f[3]
end
trajForce = zeros(length(force))
trajXForce = zeros(length(force))
trajYForce = zeros(length(force))
trajZForce = zeros(length(force))
for k = 1:length(force)
    trajForce[k] = norm(force[k])
    trajXForce[k] = force[k][1]
    trajYForce[k] = force[k][2]
    trajZForce[k] = force[k][3]
end

# Plot force
plot1 = plot(distTraj, trajForce, title="Force Over Distance Traveled", xlabel="Distance Traveled (mm)", ylabel="Force (N)", lw=3, label="Actual Trajectory Force", legend_position=:inside)
plot!(distRef, refForce, lw=3, label="Reference Trajectory Force")
plot!(distRef, 300*ones(length(distRef)), lw=3, label="Force Constraint")

plot2 = plot(distTraj, trajXForce, title="Force in X Over Distance Traveled", xlabel="Distance Traveled (mm)", ylabel="Force (N)", lw=3, label="Actual Trajectory Force", legend_position=:inside)
plot!(distRef, refXForce, lw=3, label="Reference Trajectory Force")
plot!(distRef, F_lim[1]*ones(length(distRef)), lw=3, label="Force Constraint")

plot3 = plot(distTraj, trajYForce, title="Force in Y Over Distance Traveled", xlabel="Distance Traveled (mm)", ylabel="Force (N)", lw=3, label="Actual Trajectory Force", legend_position=:bottomright)
plot!(distRef, refYForce, lw=3, label="Reference Trajectory Force")
plot!(distRef, F_lim[2]*ones(length(distRef)), lw=3, label="Force Constraint")

plot4 = plot(distTraj, trajZForce, title="Force in Z Over Distance Traveled", xlabel="Distance Traveled (mm)", ylabel="Force (N)", lw=3, label="Actual Trajectory Force", legend_position=:inside)
plot!(distRef, refZForce, lw=3, label="Reference Trajectory Force")
plot!(distRef, -1*F_lim[3]*ones(length(distRef)), lw=3, label="Force Constraint")

plot(plot1, plot2, plot3, plot4, layout = (4, 1), size=(1200, 1200), margin=5mm)
savefig(joinpath(@__DIR__,"../saved_data/plots/force.png"))


# Plot Trajectories
plot1 = plot(refX, refY, title="Actual vs Reference End-Effector Trajectory", xlabel="X Position (m)", ylabel="Y Position (m)", lw=3, label="Reference Trajectory", legend_position=:bottomleft)
plot!(trajX, trajY, lw=3, label="Actual Trajectory")
plot2 = plot(refX, refZ, title="Actual vs Reference End-Effector Trajectory", xlabel="X Position (m)", ylabel="Z Position (m)", lw=3, label="Reference Trajectory", legend_position=:bottomleft)
plot!(trajX, trajZ, lw=3, label="Actual Trajectory")

plot(plot1, plot2, layout = (2, 1), size=(1200, 1200), margin=5mm)
savefig(joinpath(@__DIR__,"../saved_data/plots/traj.png"))

# Plot Traj errors
plot(distTraj, 1000*trajError, title="Error of Actual vs Reference End-Effector Trajectory", xlabel="Distance Traveled (mm)", ylabel="Norm of Error (mm)", lw=3, legend=false)
savefig(joinpath(@__DIR__,"../saved_data/plots/ee_error.png"))

# Plot Cartesian Velocities
plot(distTraj, 1000*norm.(speed), title="Actual vs Reference End-Effector Speed", xlabel="Distance Traveled (mm)", ylabel="End-Effector Speed (mm/s)", lw=3, label="Actual Trajectory", legend_position=:bottomleft, margin=5mm)
plot!(distRef, 1000*norm.(refSpeed), lw=3, label="Reference Trajectory")
plot!(distRef, ones(length(refSpeed)), lw=3, label="Maximum Speed")
savefig(joinpath(@__DIR__,"../saved_data/plots/ee_speed.png"))

# Plot Joint Velocities
plot1 = plot(distTraj, jointVelsTraj[:, 1], title="Actual vs Reference Joint 1 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 1 Velocity (rad/s)", lw=3, label="Actual", legend_position=:inside, margin=5mm)
plot!(distRef, jointVelsRef[:, 1], lw=3, label="Reference")
plot2 = plot(distTraj, jointVelsTraj[:, 2], title="Actual vs Reference Joint 2 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 2 Velocity (rad/s)", lw=3, label="Actual", legend_position=:inside, margin=5mm)
plot!(distRef, jointVelsRef[:, 2], lw=3, label="Reference")
plot3 = plot(distTraj, jointVelsTraj[:, 3], title="Actual vs Reference Joint 3 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 3 Velocity (rad/s)", lw=3, label="Actual", legend_position=:inside, margin=5mm)
plot!(distRef, jointVelsRef[:, 3], lw=3, label="Reference")
plot4 = plot(distTraj, jointVelsTraj[:, 4], title="Actual vs Reference Joint 4 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 4 Velocity (rad/s)", lw=3, label="Actual", legend_position=:inside, margin=5mm)
plot!(distRef, jointVelsRef[:, 4], lw=3, label="Reference")
plot5 = plot(distTraj, jointVelsTraj[:, 5], title="Actual vs Reference Joint 5 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 5 Velocity (rad/s)", lw=3, label="Actual", legend_position=:inside, margin=5mm)
plot!(distRef, jointVelsRef[:, 5], lw=3, label="Reference")
plot6 = plot(distTraj, jointVelsTraj[:, 6], title="Actual vs Reference Joint 6 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 6 Velocity (rad/s)", lw=3, label="Actual", legend_position=:inside, margin=5mm)
plot!(distRef, jointVelsRef[:, 6], lw=3, label="Reference")
plot7 = plot(distTraj, jointVelsTraj[:, 7], title="Actual vs Reference Joint 7 Speed", xlabel="Distance Traveled (mm)", ylabel="Joint 7 Velocity (rad/s)", lw=3, label="Actual", legend_position=:bottomleft, margin=5mm)
plot!(distRef, jointVelsRef[:, 7], lw=3, label="Reference")

plot(plot1, plot2, plot3, plot4, plot5, plot6, plot7, layout = (7, 1), size=(1200, 2000), left_margin=20mm)
savefig(joinpath(@__DIR__,"../saved_data/plots/joint_speed.png"))

# Plot Joint Torques
plot(distTraj[1:end-1], jointTorquesTraj[:, 1], title="Joint Torques Over Trajectory (No Load)", xlabel="Distance Traveled (mm)", ylabel="Joint Torques (Nm)", lw=3, label="Joint 1", legend_position=:inside, margin=5mm)
plot!(distTraj[1:end-1], jointTorquesTraj[:, 2], lw=3, label="Joint 2")
plot!(distTraj[1:end-1], jointTorquesTraj[:, 3], lw=3, label="Joint 3")
plot!(distTraj[1:end-1], jointTorquesTraj[:, 4], lw=3, label="Joint 4")
plot!(distTraj[1:end-1], jointTorquesTraj[:, 5], lw=3, label="Joint 5")
plot!(distTraj[1:end-1], jointTorquesTraj[:, 6], lw=3, label="Joint 6")
plot!(distTraj[1:end-1], jointTorquesTraj[:, 7], lw=3, label="Joint 7")
savefig(joinpath(@__DIR__,"../saved_data/plots/joint_torque_noload.png"))

totalTorque = jointTorquesTraj + jointTorquesFromForce
plot(distTraj[1:end-1], totalTorque[:, 1], title="Joint Torques Over Trajectory (Load)", xlabel="Distance Traveled (mm)", ylabel="Joint Torques (Nm)", lw=3, label="Joint 1", legend_position=:topright, margin=5mm)
plot!(distTraj[1:end-1], totalTorque[:, 2], lw=3, label="Joint 2")
plot!(distTraj[1:end-1], totalTorque[:, 3], lw=3, label="Joint 3")
plot!(distTraj[1:end-1], totalTorque[:, 4], lw=3, label="Joint 4")
plot!(distTraj[1:end-1], totalTorque[:, 5], lw=3, label="Joint 5")
plot!(distTraj[1:end-1], totalTorque[:, 6], lw=3, label="Joint 6")
plot!(distTraj[1:end-1], totalTorque[:, 7], lw=3, label="Joint 7")
savefig(joinpath(@__DIR__,"../saved_data/plots/joint_torque_load.png"))
