# Estimate simple harmonic oscillator with the extended Kalman filter

using GLMakie
GLMakie.activate!()

# Generate samples

using Random

rng = Xoshiro(240116)
A = 8
ϕ = 0
ω = 5
xs = Float64[]
vs = Float64[]
ϕs = Float64[]
ωs = Float64[]
As = Float64[]
for i in 1:10000
  dt = 0.01
  global A, ϕ, ω
  ω += (randn(rng, Float64)*0.5) * 16 * dt
  ω = clamp(ω, 0.5, 50)
  A += (randn(rng, Float64)*0.5) * 24 * dt * (1.5 - abs(sin(ϕ)))
  A = clamp(A, 2, 50)
  ϕ += ω * dt
  x = A * sin(ϕ)
  push!(xs, x)
  push!(vs, A * cos(ϕ))
  push!(ϕs, (ϕ + π) % (2 * π) - π)
  push!(ωs, ω)
  push!(As, A)
end

fig = Figure(size = (1200, 480))
ax = Axis(fig[1, 1])
lines!(ax, 1..length(xs), xs)
fig
wait(display(fig))

# Estimation

using LinearAlgebra

function EKF_step(x, P, z, u, F, H, Q, R)
  x1 = F * x + u
  P1 = F * P * transpose(F) + Q
  y = z - H * x1
  S = H * P1 * transpose(H) + R
  K = P1 * transpose(H) * inv(S)
  x2 = x1 + K * y
  P2 = (I - K * H) * P1
  x2, P2, K
end

# ω, A cos ϕ, A sin ϕ
x = [10; 0; 0]
P = [10 0 0;
     0 1e8 0;
     0 0 1e8]
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
  σ = 0.5
  σv = 8
  observedPos = xs[i] + randn(rngObserve, Float64) * σ
  observedVel = vs[i] + randn(rngObserve, Float64) * σv
  z = [observedPos; observedVel]
  ω = x[1]
  sin_ωdt = sin(ω * dt)
  cos_ωdt = cos(ω * dt)
  F = [1 0 0;
       dt*(-x[3]*cos_ωdt - x[2]*sin_ωdt) cos_ωdt -sin_ωdt;
       dt*( x[2]*cos_ωdt - x[3]*sin_ωdt) sin_ωdt  cos_ωdt]
  u = [0.0; 0; 0]
  H = [0.0 0 1; 0 1 0]
  Q = [1 0 0;
       0 0.01 0;
       0 0 0.01] * dt^2
  R = [0.04*sqrt(x[2]^2+x[3]^2)/10 0;
       0 0.32]
  x, P, K = EKF_step(x, P, z, u, F, H, Q, R)
  push!(xsObserved, observedPos)
  push!(xsFiltered, x[3])
  push!(amplFiltered, sqrt(x[2]^2 + x[3]^2))
  push!(freqFiltered, x[1])
  push!(noise, observedPos - xs[i])
  push!(residue, x[3] - xs[i])
  push!(phasesFiltered, atan(x[3], x[2]))
end

N0 = N
M = 1
# M, N = ...
fig = Figure(size = (1200, 720))
ax = Axis(fig[1, 1])
l1 = lines!(ax, M..N, xs[1:N0][M:N])
l2 = lines!(ax, M..N, xsObserved[M:N])
l3 = lines!(ax, M..N, clamp.(xsFiltered, -50, 50)[M:N])
l4 = lines!(ax, M..N, clamp.(amplFiltered, -50, 50)[M:N])
l5 = lines!(ax, M..N, clamp.(freqFiltered, -50, 50)[M:N])
l6 = lines!(ax, M..N, noise[M:N])
l7 = lines!(ax, M..N, clamp.(residue, -50, 50)[M:N])
Legend(fig[1, 2], [l1, l2, l3, l4, l5, l6, l7], ["true", "observed", "filtered", "filtered am", "filtered avel", "noise", "residue"])
ax = Axis(fig[2, 1])
l8 = lines!(ax, M..N, ϕs[M:N])
l9 = lines!(ax, M..N, phasesFiltered[M:N])
l10 = lines!(ax, M..N, clamp.((phasesFiltered .- ϕs[1:N0]) .* 5, -π, π)[M:N])
Legend(fig[2, 2], [l8, l9, l10], ["true ph", "filtered ph", "diff ×5"])
phDiffValid = filter(x -> abs(x) < 1.75π, phasesFiltered .- ϕs[1:N0])
phRmse = norm(phDiffValid, 2) / sqrt(length(phDiffValid))
println("ph RMSE = ", phRmse, " rad = ", phRmse / 2π, " rev")
println("confidence = ", length(phDiffValid) / N0)

ax = Axis(fig[3, 1])
l11 = lines!(ax, M..N, As[M:N])
l4 = lines!(ax, M..N, amplFiltered[M:N])
l12 = lines!(ax, M..N, (ωs ./ 2)[M:N])
l5 = lines!(ax, M..N, clamp.(freqFiltered, -10, 50)[M:N])
Legend(fig[3, 2], [l11, l4, l12, l5], ["true am", "filtered am", "true avel", "filtered avel"])
fig
wait(display(fig))
