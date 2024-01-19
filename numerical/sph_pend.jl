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

θ = 0.4
ϕ = 0.0
θd = 0.0
ϕd = 0.5
k = 1.5

x = [θ, ϕ, θd, ϕd]
p = Observable(Point3f[])
vs = Observable(repeat([0.0], 5000))
pLast = Observable(Point3f(0))

fig = Figure(size = (1200, 1200))
ax1 = Axis3(fig[1, 1])
scatter!(ax1, [Point3f(-1), Point3f(0), Point3f(1)])
scatter!(ax1, @lift([$pLast]))
lines!(ax1, p)

ax2 = Axis(fig[1, 2])
lines!(ax2, vs)
display(fig)

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
  pCur = Point3f(sin(θ)*cos(ϕ), sin(θ)*sin(ϕ), -cos(θ))
  push!(p[], pCur); p[] = p[]
  pLast[] = pCur
  vs[][i] = sqrt(θd^2 + sin(θ)^2 * ϕd^2); vs[] = vs[]
  sleep(0.001)
end
