# Simulate a spherical pendulum

using GLMakie
GLMakie.activate!()

RK4(f, x, dt) = begin
  k1 = f(x)
  k2 = f(x + k1 * (0.5 * dt))
  k3 = f(x + k2 * (0.5 * dt))
  k4 = f(x + k3 * dt)
  (k1 + 2 * k2 + 2 * k3 + k4) / 6 * dt
end

θ = 0.1
ϕ = 0.0
θd = 0.0
ϕd = 0.01
k = 1.5

x = [θ, ϕ, θd, ϕd]
p = Point3f[]
θs = Float32[]
ϕs = Float32[]
vs = Float32[]
xs = Float32[]
ys = Float32[]
zs = Float32[]
pLast = Point3f(0)

using LinearAlgebra: norm

for i in 1:10000
  dt = 0.001
  global x
  x += RK4(x, dt) do(x)
    θ, ϕ, θd, ϕd = x
    θdd = sin(θ) * cos(θ) * ϕd^2 - k * sin(θ)
    ϕdd = -2 * θd * ϕd * cot(θ)
    [θd, ϕd, θdd, ϕdd]
  end
  local θ, ϕ, θd, ϕd = x
  pCur = Point3f(sin(θ)*cos(ϕ), sin(θ)*sin(ϕ), -cos(θ))
  push!(p, pCur)
  push!(θs, θ)
  push!(ϕs, ϕ % π)
  # push!(vs, sqrt(θd^2 + sin(θ)^2 * ϕd^2))
  push!(vs, if i == 1 0 else norm(pCur .- pLast, 2) / dt end)
  global pLast = pCur
  push!(xs, pCur[1])
  push!(ys, pCur[2])
  push!(zs, pCur[3])
end

fig = Figure(size = (1200, 1200))
ax1 = Axis3(fig[1, 1])
scatter!(ax1, [Point3f(-1), Point3f(0), Point3f(1)])
lines!(ax1, p)

ax2 = Axis(fig[2, 1])
lθ = lines!(ax2, θs)
lϕ = lines!(ax2, ϕs)
lv = lines!(ax2, vs)
lx = lines!(ax2, xs)
ly = lines!(ax2, ys)
lz = lines!(ax2, zs)
Legend(fig[2, 2], [lθ, lϕ, lv, lx, ly, lz], ["θ", "ϕ", "v", "x", "y", "z"])

wait(display(fig))
