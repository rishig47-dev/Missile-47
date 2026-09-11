%% ========================================================================
%  MISSILE ANIMATION: POINT A -> POINT B
%  ------------------------------------------------------------------------
%  Simple, robust animation: define a START point and an END point, the
%  script computes a physics-based flight path between them (gravity,
%  drag, thrust, and guidance to steer onto the target) and animates a
%  single marker moving smoothly from A to B, leaving a trail behind it.
%
%  This version is deliberately simplified (no particle effects, no HUD
%  clutter) to be easy to read, easy to modify, and error-free.
% =========================================================================

clear; clc; close all;

%% ------------------------- 1. DEFINE START AND END POINTS -----------------
P_start = [0,     0,    0   ];   % [X, Y, Altitude] in meters
P_end   = [6000,  1200, 0   ];   % [X, Y, Altitude] in meters  (target on the ground)

launch_speed_ms = 250;            % initial speed [m/s]
launch_elev_deg = 35;             % initial launch angle [deg] (guidance corrects the rest)

%% ------------------------- 2. VEHICLE / PHYSICS SETTINGS -------------------
cfg.mass_initial_kg = 120;
cfg.mass_burnout_kg = 70;
cfg.diameter_m      = 0.20;
cfg.ref_area_m2      = pi*(cfg.diameter_m/2)^2;

cfg.thrust_profile = [ 0.0  2.0  9000;
                       2.0  8.0  1800;
                       8.0  1e6  0   ];

cfg.mach_table = [0 0.6 0.8 1.0 1.2 1.5 2.0 3.0 5.0];
cfg.cd_table   = [0.30 0.32 0.45 0.62 0.58 0.48 0.40 0.32 0.28];

cfg.g0 = 9.80665;
cfg.Re = 6371000;

cfg.N_gain              = 4;
cfg.max_lateral_accel_g = 20;
cfg.guidance_start_t    = 1.0;

cfg.dt    = 0.01;
cfg.t_max = 120;

%% ------------------------- 3. INITIAL HEADING TOWARD END POINT -------------
horiz_dir = (P_end(1:2) - P_start(1:2));
horiz_dist = norm(horiz_dir);
if horiz_dist < 1e-6
    az0 = 0;
else
    az0 = atan2d(horiz_dir(2), horiz_dir(1));
end

gamma0 = deg2rad(launch_elev_deg);
psi0   = deg2rad(az0);
vx0 = launch_speed_ms*cos(gamma0)*cos(psi0);
vy0 = launch_speed_ms*cos(gamma0)*sin(psi0);
vz0 = launch_speed_ms*sin(gamma0);

state0 = [P_start(1); P_start(2); P_start(3); vx0; vy0; vz0; cfg.mass_initial_kg];

%% ------------------------- 4. RUN THE SIMULATION (RK4) ---------------------
N = ceil(cfg.t_max/cfg.dt);
T = zeros(N,1);
Sm = zeros(N,7);
Sm(1,:) = state0';

impact_idx = N;
for k = 1:N-1
    t = T(k);
    s = Sm(k,:)';

    ds = missile_dynamics(t, s, cfg, P_end);
    k1 = ds;
    k2 = missile_dynamics(t+cfg.dt/2, s+cfg.dt/2*k1, cfg, P_end);
    k3 = missile_dynamics(t+cfg.dt/2, s+cfg.dt/2*k2, cfg, P_end);
    k4 = missile_dynamics(t+cfg.dt,   s+cfg.dt*k3,   cfg, P_end);
    s_next = s + (cfg.dt/6)*(k1+2*k2+2*k3+k4);

    T(k+1) = t + cfg.dt;
    Sm(k+1,:) = s_next';

    dist_to_end = norm(s_next(1:3) - P_end(:));
    if s_next(3) <= 0 || dist_to_end < 10
        impact_idx = k+1;
        break;
    end
end

T  = T(1:impact_idx);
Sm = Sm(1:impact_idx,:);
Pos = Sm(:,1:3);

fprintf('Flight complete in %.2f s. Final position: (%.1f, %.1f, %.1f). Distance from target: %.1f m\n', ...
    T(end), Pos(end,1), Pos(end,2), Pos(end,3), norm(Pos(end,:)-P_end));

%% ------------------------- 5. RESAMPLE FOR SMOOTH PLAYBACK -----------------
playback_fps = 30;
playback_seconds = 6;              % how long the animation should take to play, regardless of flight time
t_play = linspace(0, T(end), playback_fps*playback_seconds);
pos_play = interp1(T, Pos, t_play, 'linear');

%% ------------------------- 6. SET UP THE ANIMATION -------------------------
figure('Name','Missile Animation: Point A to Point B','Color','w');
ax = axes; hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Altitude [m]');
title('Missile Path: Start \rightarrow End');
view(3);

xlim([min(Pos(:,1))-200, max(Pos(:,1))+200]);
ylim([min(Pos(:,2))-200, max(Pos(:,2))+200]);
zlim([0, max(Pos(:,3))*1.15 + 10]);

% Mark start and end points
plot3(P_start(1), P_start(2), P_start(3), 'go', 'MarkerFaceColor','g', 'MarkerSize',10);
plot3(P_end(1),   P_end(2),   P_end(3),   'rp', 'MarkerFaceColor','r', 'MarkerSize',14);
text(P_start(1), P_start(2), P_start(3), '  START', 'Color','g','FontWeight','bold');
text(P_end(1),   P_end(2),   P_end(3),   '  END',   'Color','r','FontWeight','bold');

% Trail line and moving marker
h_trail  = animatedline(ax, 'Color',[0.1 0.4 0.9], 'LineWidth', 2);
h_marker = plot3(ax, pos_play(1,1), pos_play(1,2), pos_play(1,3), ...
    'o', 'MarkerSize', 10, 'MarkerFaceColor', [1 0.5 0], 'MarkerEdgeColor','k');

%% ------------------------- 7. PLAY THE ANIMATION ---------------------------
for i = 1:size(pos_play,1)
    addpoints(h_trail, pos_play(i,1), pos_play(i,2), pos_play(i,3));
    set(h_marker, 'XData', pos_play(i,1), 'YData', pos_play(i,2), 'ZData', pos_play(i,3));
    drawnow limitrate;
    pause(1/playback_fps);
end

disp('Animation finished: missile traveled from START to END.');

%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================

function ds = missile_dynamics(t, s, cfg, target_pos)
    pos = s(1:3); vel = s(4:6); m = s(7); alt = max(pos(3),0);

    [rho, a_sound] = std_atmosphere(alt);
    V = max(norm(vel), 1e-6);
    mach = V / a_sound;

    Cd = interp1(cfg.mach_table, cfg.cd_table, mach, 'linear', 'extrap');
    q = 0.5*rho*V^2;
    F_drag = -(q*Cd*cfg.ref_area_m2) * (vel(:)/V);

    thrust_mag = 0;
    tp = cfg.thrust_profile;
    for i = 1:size(tp,1)
        if t >= tp(i,1) && t < tp(i,2)
            thrust_mag = tp(i,3);
            break;
        end
    end
    speed_dir = vel(:)/max(norm(vel),1e-6);
    F_thrust = thrust_mag * speed_dir;

    F_guidance = [0;0;0];
    if t >= cfg.guidance_start_t
        r_vec = target_pos(:) - pos(:);
        r_mag = norm(r_vec);
        if r_mag > 1e-3
            v_target_rel = -vel(:);   % stationary target
            los_rate_vec = cross(r_vec, v_target_rel) / (r_mag^2);
            Vc = -dot(r_vec, v_target_rel)/r_mag;
            v_dir = vel(:)/max(norm(vel),1e-6);
            a_cmd_vec = cfg.N_gain * Vc * cross(los_rate_vec, v_dir);
            a_cmd_mag = norm(a_cmd_vec);
            max_a = cfg.max_lateral_accel_g * cfg.g0;
            if a_cmd_mag > max_a
                a_cmd_vec = a_cmd_vec * (max_a/a_cmd_mag);
            end
            F_guidance = m * a_cmd_vec;
        end
    end

    g_alt = cfg.g0 * (cfg.Re/(cfg.Re+alt))^2;
    F_gravity = [0;0;-m*g_alt];

    F_total = F_thrust + F_drag + F_gravity + F_guidance;
    accel = F_total/m;

    if thrust_mag > 0 && m > cfg.mass_burnout_kg
        burn_duration = tp(1,2) + (tp(2,2)-tp(2,1));
        total_prop = cfg.mass_initial_kg - cfg.mass_burnout_kg;
        mdot = -total_prop/burn_duration;
    else
        mdot = 0;
    end

    ds = zeros(7,1);
    ds(1:3) = vel;
    ds(4:6) = accel;
    ds(7) = mdot;
end

function [rho, a_sound] = std_atmosphere(h)
    h = max(h,0);
    R = 287.05287; g0 = 9.80665; gamma_air = 1.4;
    layers = [0,288.15,-0.0065,101325.0;
              11000,216.65,0.0,22632.06;
              20000,216.65,0.001,5474.889;
              32000,228.65,0.0028,868.0187;
              47000,270.65,0.0,110.9063];
    idx = find(h >= layers(:,1), 1, 'last'); if isempty(idx), idx=1; end
    hb = layers(idx,1); Tb = layers(idx,2); L = layers(idx,3); Pb = layers(idx,4);
    if abs(L) > 1e-12
        T = Tb + L*(h-hb);
        P = Pb*(T/Tb)^(-g0/(L*R));
    else
        T = Tb;
        P = Pb*exp(-g0*(h-hb)/(R*Tb));
    end
    rho = P/(R*T);
    a_sound = sqrt(gamma_air*R*T);
end
