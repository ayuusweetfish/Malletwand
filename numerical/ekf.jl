# Estimate simple harmonic oscillator with the extended Kalman filter

using GLMakie
GLMakie.activate!()

# Generate samples

using Random

rng = Xoshiro(240116)
A = 10
ϕ = 0
ω = 1
xs = Float64[]
ϕs = Float64[]
for i in 1:20000
  dt = 0.01
  global A, ϕ, ω
  ω += (rand(rng, Float64) * 2 - 1) * 0.8 * dt
  ω = clamp(ω, 0.2, 2)
  A += (rand(rng, Float64) * 2 - 1) * 4 * dt * (1.5 - abs(sin(ϕ)))
  A = clamp(A, 2, 20)
  ϕ += ω * dt
  x = A * sin(ϕ)
  push!(xs, x)
  push!(ϕs, (ϕ + π) % (2 * π) - π)
end

fig = Figure()
ax = Axis(fig[1, 1])
lines!(ax, 1..length(xs), xs)
fig
wait(display(fig))

# Estimation

using LinearAlgebra

function EKF_step(x, P, z, F, H, Q, R)
  x1 = F * x
  P1 = F * P * transpose(F) + Q
  y = z - H * x1
  S = H * P1 * transpose(H) + R
  K = P1 * transpose(H) * inv(S)
  x2 = x1 + K * y
  P2 = (I - K * H) * P1
  x2, P2, K
end

# ω, A cos ϕ, A sin ϕ
x = [1; 10; 0]
P = I * 1e4
rngObserve = Xoshiro(24011620)
xsObserved = Float64[]
xsFiltered = Float64[]
amplFiltered = Float64[]
freqFiltered = Float64[]
noise = Float64[]
residue = Float64[]
phasesFiltered = Float64[]
N = length(xs)
for i in 1:N
  dt = 0.01
  global x, P
  local ω
  observedPos = xs[i] + (randn(rngObserve, Float64) * 2 - 1) * 0.3
  z = [observedPos]
  ω = x[1]
  F = [1 0 0;
       -dt*x[3] cos(ω*dt) -sin(ω*dt);
        dt*x[2] sin(ω*dt)  cos(ω*dt)]
  H = [0 0 1]
  Q = [0.001 0 0;
       0 0.1 0;
       0 0 0.1] * dt^2
  R = [0.09]
  x, P, K = EKF_step(x, P, z, F, H, Q, R)
  push!(xsObserved, observedPos)
  push!(xsFiltered, x[3])
  push!(amplFiltered, sqrt(x[2]^2 + x[3]^2))
  push!(freqFiltered, x[1])
  push!(noise, observedPos - xs[i])
  push!(residue, x[3] - xs[i])
  push!(phasesFiltered, atan(x[3], x[2]))
end

fig = Figure(size = (1200, 480))
ax = Axis(fig[1, 1])
l1 = lines!(ax, 1..N, xs[1:N])
l2 = lines!(ax, 1..N, xsObserved)
l3 = lines!(ax, 1..N, clamp.(xsFiltered, -18, 18))
l4 = lines!(ax, 1..N, clamp.(amplFiltered, -18, 18))
l5 = lines!(ax, 1..N, clamp.(freqFiltered .* 10, -18, 18))
l6 = lines!(ax, 1..N, noise)
l7 = lines!(ax, 1..N, clamp.(residue, -18, 18))
Legend(fig[1, 2], [l1, l2, l3, l4, l5, l6, l7], ["true", "observed", "filtered", "filtered am", "filtered avel ×10", "noise", "residue"])
ax = Axis(fig[2, 1])
l8 = lines!(ax, 1..N, ϕs)
l9 = lines!(ax, 1..N, phasesFiltered)
l10 = lines!(ax, 1..N, clamp.((phasesFiltered .- ϕs[1:N]) .* 5, -π, π))
Legend(fig[2, 2], [l8, l9, l10], ["true ph", "filtered ph", "diff ×5"])
phDiffValid = filter(x -> abs(x) < π/2, phasesFiltered .- ϕs[1:N])
println("ph RMSE = ", norm(phDiffValid, 2) / sqrt(length(phDiffValid)), " rad")
fig
wait(display(fig))
