using GLMakie
GLMakie.activate!()

k = 1.5
pendulumDeriv(θ, dθ) = Point2f(dθ, -k * sin(θ))

fig = Figure(backgroundcolor = :white)

# Phase portrait of a simple undamped pendulum
ax = Axis(
  fig[1, 1],
  xlabel = "θ", xticks = MultiplesTicks(4, π, "π"),
  ylabel = "dθ/dt",
  backgroundcolor = :transparent)
streamplot!(
  ax, pendulumDeriv,
  -π * 1.5 .. π * 1.5, -4.5 .. 4.5,
  colormap = :acton, arrow_size = 10)
fig

# Boundary between swinging and continuous rotation
critThetaDeriv(θ) = sqrt(2k * (cos(θ) + 1))
x = range(-π * 1.5 .. π * 1.5, 100)
y = critThetaDeriv.(x)
lines!(ax, x, y, color = :tomato, linewidth = 3)
fig

wait(display(fig))

# Solve time (or phase, or progress) from angle

#=
sqrt(1/0.8) * EllipticF(pi/2, sin(0.6)^2) -- 1.92868
integrate from 0 to 1.2, 1/sqrt(1.6*(cos(t) - cos(1.2))) dt -- 1.92868
sqrt(2/(0.8*(1-cos(1.2)))) * EllipticF(0.6, 1/sin(0.6)^2) -- 1.92868

integrate from 0 to 0.8, 1/sqrt(1.6*(cos(t) - cos(1.2))) dt -- 0.876
sqrt(2/(0.8*(1-cos(1.2)))) * EllipticF(0.4, 1/sin(0.6)^2) -- 0.876

integrate from 0 to 0.7, 1/sqrt(1.6*(cos(t) - cos(1.2))) dt -- 0.74572
integrate from 0 to 0.7, 1/sqrt(1.6) * 1/sqrt(((1-cos(1.2)) * (1 - 2/(1-cos(1.2))*sin(t/2)^2))) dt -- 0.74572
integrate from 0 to 0.7, 1/sqrt(1.6*(1-cos(1.2))) * 1/sqrt(1 - sin(t/2)^2 / sin(0.6)^2) dt -- 0.74572
sqrt(2/(0.8*(1-cos(1.2)))) * EllipticF(0.35, 1/sin(0.6)^2) -- 0.74572
=#

# Coloured heatmap for phase estimation

# https://github.com/nolta/Elliptic.jl/blob/master/src/slatec.jl
# https://www.netlib.org/slatec/
# SLATEC, Carlson, B. C. et al.

function DRF(X, Y, Z)
    ERRTOL = (4.0*(eps(Float64)/2))^(1.0/6.0)

    local MU, XNDEV, YNDEV, ZNDEV
    XN, YN, ZN = X, Y, Z

    while true
        MU = (XN+YN+ZN)/3.0
        XNDEV = 2.0 - (MU+XN)/MU
        YNDEV = 2.0 - (MU+YN)/MU
        ZNDEV = 2.0 - (MU+ZN)/MU
        EPSLON = max(abs(XNDEV), abs(YNDEV), abs(ZNDEV))
        if EPSLON < ERRTOL break end
        XNROOT = sqrt(XN)
        YNROOT = sqrt(YN)
        ZNROOT = sqrt(ZN)
        LAMDA = XNROOT*(YNROOT+ZNROOT) + YNROOT*ZNROOT
        XN = (XN+LAMDA)*0.250
        YN = (YN+LAMDA)*0.250
        ZN = (ZN+LAMDA)*0.250
    end

    E2 = XNDEV*YNDEV - ZNDEV*ZNDEV
    E3 = XNDEV*YNDEV*ZNDEV
    S  = 1.0 + ((1.0/24.0)*E2 - 0.10 - (3.0/44.0)*E3)*E2 + (1.0/14.0)*E3
    ans = S/sqrt(MU)

    return ans
end

function ellipticF(sin_ϕ, m)
  if 1 - m*sin_ϕ^2 < -1e-8 return 0.0 end
  drf = DRF(1 - sin_ϕ^2, max(0, 1 - m*sin_ϕ^2), 1.0)
  sin_ϕ*drf
end
# ellipticF(sin(0.3), 0.8) == 0.30365239221539

function progressHalf(θ, dθ)
  u = cos(θ) - dθ * dθ / (2 * k)
  if abs(u) > 1 return 0.0 end
  # θ_0 = acos(u), t = sin(θ_0/2)^2
  t = (1 - u) / 2
  T = ellipticF(sin(θ/2), 1 / t)
  T_0 = ellipticF(sqrt(t), 1 / t)
  return T / T_0  # [-1, 1]
end

function progress(θ, dθ)
  if abs(dθ) >= critThetaDeriv(θ) return 0.5 end
  if dθ >= 0
    return progressHalf(θ, dθ) * 0.25 + 0.25
  else
    return progressHalf(θ, -dθ) * -0.25 + 0.75
  end
end

x = range(-π .. π, 1000)
y = range(-4.5 .. 4.5, 1000)
heatmap!(x, y, progress, colormap = Reverse(:lightrainbow))
fig

# Difference from arctan
function diffArctan(θ, dθ)
  if abs(dθ) >= critThetaDeriv(θ) return 0.0 end
  progress(θ, dθ) - (atan(dθ, θ) / pi * -0.5 + 0.5)
end
hm = heatmap!(x, y, diffArctan, colormap = :vik)
Colorbar(fig[1, 2], hm)
fig

wait(display(fig))

# Close-up
fig = Figure(backgroundcolor = :white)
ax = Axis(
  fig[1, 1],
  xlabel = "θ", xticks = MultiplesTicks(4, π, "π"),
  ylabel = "dθ/dt",
  backgroundcolor = :transparent)
θ_lim = π * 0.5
dθ_lim = 2.0
x = range(-θ_lim .. θ_lim, 1000)
y = range(-dθ_lim .. dθ_lim, 1000)
function diffArctanCut(θ, dθ)
  if (θ/θ_lim)^2 + (dθ/dθ_lim)^2 >= 1 return 0.0 end
  diffArctan(θ, dθ)
end
hm = heatmap!(x, y, diffArctanCut, colormap = Reverse(:RdYlBu_3))
Colorbar(fig[1, 2], hm)
fig

wait(display(fig))
