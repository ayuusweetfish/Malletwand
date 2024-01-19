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
ϕd = 0.1
k = 1.5

x = [θ, ϕ, θd, ϕd]
p = Point3f[]
vs = Float32[]

for i in 1:5000
  dt = 0.01
  global x
  x += RK4(x, dt) do(x)
    θ, ϕ, θd, ϕd = x
    θdd = sin(θ) * cos(θ) * ϕd^2 - k * sin(θ)
    ϕdd = -2 * θd * ϕd * cot(θ)
    [θd, ϕd, θdd, ϕdd]
  end
  local θ, ϕ, θd, ϕd = x
  push!(p, Point3f(sin(θ)*cos(ϕ), sin(θ)*sin(ϕ), -cos(θ)))
  push!(vs, sqrt(θd^2 + sin(θ)^2 * ϕd^2))
end

fig = Figure(size = (1200, 1200))
ax1 = Axis3(fig[1, 1])
scatter!(ax1, [Point3f(-1), Point3f(0), Point3f(1)])
lines!(ax1, p)

ax2 = Axis(fig[2, 1])
lines!(ax2, vs)
wait(display(fig))
