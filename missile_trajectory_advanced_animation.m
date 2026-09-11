%% ========================================================================
%  FULLY ADVANCED MISSILE TRAJECTORY ANIMATION
%  ------------------------------------------------------------------------
%  Standalone script: runs the 3-DOF point-mass flight simulation, then
%  renders a cinematic-style animation featuring:
%    - Oriented 3D missile body (nose always points along velocity vector)
%    - Exhaust plume particles during powered flight, fading with age
%    - Motion trail colored by instantaneous speed (Mach-shaded)
%    - Dynamic chase camera that follows and banks with the missile
%    - Ground plane grid + drop-shadow marker beneath the missile
%    - Live HUD overlay: time, altitude, speed, Mach, range-to-target,
%      guidance g-command
%    - Synced mini-plots (altitude & speed vs time) with moving cursor
%    - Optional MP4 export via VideoWriter
%
%  This is a generic teaching/visualization model (simplified point-mass
%  aero/atmosphere/guidance), not a real weapon system design.
% =========================================================================

clear; clc; close all;

%% ------------------------- 1. CONFIGURATION ------------------------------
cfg = struct();
cfg.launch_alt_m      = 0;
cfg.launch_speed_ms   = 50;
cfg.launch_elev_deg   = 45;
cfg.launch_az_deg     = 0;

cfg.mass_initial_kg   = 120;
cfg.mass_burnout_kg   = 70;
cfg.diameter_m        = 0.20;
cfg.ref_area_m2       = pi*(cfg.diameter_m/2)^2;

cfg.thrust_profile = [ 0.0  2.0  9000;
                       2.0  8.0  1800;
                       8.0  1e6  0   ];

cfg.mach_table = [0 0.6 0.8 1.0 1.2 1.5 2.0 3.0 5.0];
cfg.cd_table   = [0.30 0.32 0.45 0.62 0.58 0.48 0.40 0.32 0.28];

cfg.g0            = 9.80665;
cfg.Re            = 6371000;
cfg.wind_NED      = [5, 2, 0];
cfg.gust_std_ms   = 1.0;
cfg.use_gusts     = true;

cfg.guidance_on          = true;
cfg.N_gain               = 4;
cfg.max_lateral_accel_g  = 20;
cfg.guidance_start_t     = 3.0;

cfg.target_pos0_m   = [8000, 1500, 1500];
cfg.target_vel_ms   = [-40, 0, 0];

cfg.dt          = 0.01;
cfg.t_max       = 120;
cfg.ground_alt  = 0;

% --- Animation-specific settings ---
anim = struct();
anim.playback_fps      = 30;      % output frame rate
anim.speedup            = 4;      % simulated seconds per real second
anim.trail_length_s     = 6;      % seconds of visible trail
anim.plume_lifetime_s   = 1.2;    % seconds a plume particle stays visible
anim.plume_emit_every   = 2;      % emit a plume particle every N frames (while thrusting)
anim.export_video       = false;  % set true to save an MP4
anim.video_filename     = 'missile_trajectory_advanced.mp4';
anim.missile_length_m   = 220;    % visual scale of missile glyph (exaggerated for visibility)
anim.missile_width_m    = 55;

%% ------------------------- 2. RUN SIMULATION ------------------------------
v0 = cfg.launch_speed_ms;
gamma0 = deg2rad(cfg.launch_elev_deg);
psi0   = deg2rad(cfg.launch_az_deg);
vx0 = v0*cos(gamma0)*cos(psi0);
vy0 = v0*cos(gamma0)*sin(psi0);
vz0 = v0*sin(gamma0);
state0 = [0; 0; cfg.launch_alt_m; vx0; vy0; vz0; cfg.mass_initial_kg];

N = ceil(cfg.t_max/cfg.dt);
T = zeros(N,1); S = zeros(N,7);
Mach_hist = zeros(N,1); Accel_cmd_hist = zeros(N,1);
Target_hist = zeros(N,3);
S(1,:) = state0';

impact_idx = N; hit_target = false;
for k = 1:N-1
    t = T(k); s = S(k,:)';
    target_pos = cfg.target_pos0_m(:) + cfg.target_vel_ms(:)*t;
    Target_hist(k,:) = target_pos';

    [ds, diag_out] = missile_dynamics(t, s, cfg, target_pos);
    Mach_hist(k) = diag_out.mach;
    Accel_cmd_hist(k) = diag_out.accel_cmd_g;

    k1 = ds;
    [k2,~] = missile_dynamics(t+cfg.dt/2, s+cfg.dt/2*k1, cfg, target_pos);
    [k3,~] = missile_dynamics(t+cfg.dt/2, s+cfg.dt/2*k2, cfg, target_pos);
    [k4,~] = missile_dynamics(t+cfg.dt,   s+cfg.dt*k3,   cfg, target_pos);
    s_next = s + (cfg.dt/6)*(k1+2*k2+2*k3+k4);

    T(k+1) = t+cfg.dt; S(k+1,:) = s_next';

    if s_next(3) <= cfg.ground_alt
        impact_idx = k+1; break;
    end
    if norm(s_next(1:3)-target_pos) < 5.0 && t > cfg.guidance_start_t
        impact_idx = k+1; hit_target = true; break;
    end
end

T = T(1:impact_idx); S = S(1:impact_idx,:);
Mach_hist = Mach_hist(1:impact_idx); Accel_cmd_hist = Accel_cmd_hist(1:impact_idx);
Target_hist = Target_hist(1:impact_idx,:);
Target_hist(end,:) = (cfg.target_pos0_m(:) + cfg.target_vel_ms(:)*T(end))';

Speed_hist = sqrt(sum(S(:,4:6).^2,2));
Range_to_target = sqrt(sum((S(:,1:3)-Target_hist).^2,2));

if hit_target
    outcome_str = 'INTERCEPT';
else
    outcome_str = 'IMPACT/END';
end
fprintf('Simulation complete: %d steps, flight time %.2f s, outcome: %s\n', ...
    impact_idx, T(end), outcome_str);

%% ------------------------- 3. RESAMPLE FOR PLAYBACK -----------------------
% Convert simulated timeline to a fixed-fps playback timeline
playback_dt = anim.speedup/anim.playback_fps;
t_play = 0:playback_dt:T(end);
Nf = numel(t_play);

pos_play   = interp1(T, S(:,1:3), t_play, 'linear');
vel_play   = interp1(T, S(:,4:6), t_play, 'linear');
mach_play  = interp1(T, Mach_hist, t_play, 'linear');
accel_play = interp1(T, Accel_cmd_hist, t_play, 'linear');
speed_play = interp1(T, Speed_hist, t_play, 'linear');
tgt_play   = interp1(T, Target_hist, t_play, 'linear');
rng_play   = interp1(T, Range_to_target, t_play, 'linear');

% Determine thrust-on flag per playback frame (for plume emission)
thrust_on_play = false(Nf,1);
for i = 1:Nf
    tt = t_play(i);
    for r = 1:size(cfg.thrust_profile,1)
        if tt >= cfg.thrust_profile(r,1) && tt < cfg.thrust_profile(r,2) && cfg.thrust_profile(r,3) > 0
            thrust_on_play(i) = true;
        end
    end
end

%% ------------------------- 4. FIGURE / AXES SETUP -------------------------
fig = figure('Name','Advanced Missile Trajectory Animation','Color',[0.05 0.07 0.10], ...
             'Position',[80 80 1280 780]);

% Main 3D chase view (large, left)
ax3d = axes('Parent',fig,'Position',[0.03 0.06 0.68 0.90]);
set(ax3d,'Color',[0.05 0.07 0.10],'XColor','w','YColor','w','ZColor','w');
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
xlabel(ax3d,'X [m]','Color','w'); ylabel(ax3d,'Y [m]','Color','w'); zlabel(ax3d,'Alt [m]','Color','w');
title(ax3d,'Missile Trajectory - Chase View','Color','w','FontWeight','bold');

maxZ = max(pos_play(:,3))*1.15 + 50;
minX = min([pos_play(:,1); tgt_play(:,1)]) - 500;
maxX = max([pos_play(:,1); tgt_play(:,1)]) + 500;
minY = min([pos_play(:,2); tgt_play(:,2)]) - 500;
maxY = max([pos_play(:,2); tgt_play(:,2)]) + 500;

% Ground grid patch
[gx,gy] = meshgrid(linspace(minX,maxX,20), linspace(minY,maxY,20));
gz = zeros(size(gx));
surf(ax3d, gx, gy, gz, 'FaceColor',[0.10 0.14 0.18], 'FaceAlpha',0.6, ...
     'EdgeColor',[0.25 0.30 0.35], 'EdgeAlpha',0.5, 'Parent',ax3d);

% Target marker (updates each frame)
h_target = plot3(ax3d, tgt_play(1,1), tgt_play(1,2), tgt_play(1,3), ...
    'o','MarkerSize',10,'MarkerFaceColor',[1 0.2 0.2],'MarkerEdgeColor','w','LineWidth',1.2);
h_target_trail = animatedline(ax3d,'Color',[1 0.3 0.3 0.5],'LineStyle','--','LineWidth',1.2);

% Missile trail (colored by speed) -- implemented as a patch line with vertex colors
h_trail = surface(ax3d,'XData',[NaN NaN;NaN NaN],'YData',[NaN NaN;NaN NaN], ...
    'ZData',[NaN NaN;NaN NaN],'CData',[NaN NaN;NaN NaN], ...
    'FaceColor','none','EdgeColor','interp','LineWidth',3);
colormap(ax3d, jet);
cb = colorbar(ax3d,'Color','w'); cb.Label.String = 'Speed [m/s]'; cb.Label.Color='w';
caxis(ax3d, [0 max(speed_play)*1.05]);

% Missile body (patch, updated via vertex transform each frame)
missile_patch = patch(ax3d, 'Vertices', zeros(5,3), 'Faces', [1 2 3;1 3 4;1 4 5;1 5 2], ...
    'FaceColor',[0.85 0.85 0.9],'EdgeColor','k','FaceLighting','gouraud');
camlight(ax3d,'headlight'); lighting(ax3d,'gouraud');

% Ground shadow marker beneath missile
h_shadow = plot3(ax3d, pos_play(1,1), pos_play(1,2), 0, 'o', ...
    'MarkerSize',6,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor','none');

% Exhaust plume particle system (preallocate)
max_particles = 400;
plume_pos = nan(max_particles,3);
plume_age = inf(max_particles,3);
plume_age = inf(max_particles,1);
h_plume = scatter3(ax3d, nan, nan, nan, 20, [1 0.6 0.1], 'filled', 'MarkerFaceAlpha',0.6);

xlim(ax3d,[minX maxX]); ylim(ax3d,[minY maxY]); zlim(ax3d,[0 maxZ]);
view(ax3d, -35, 20);

% HUD text
hud = uicontrol('Style','text','Parent',fig,'Units','normalized', ...
    'Position',[0.735 0.68 0.25 0.28],'BackgroundColor',[0.05 0.07 0.10], ...
    'ForegroundColor',[0.2 1 0.4],'FontName','Consolas','FontSize',11, ...
    'HorizontalAlignment','left','String','');

% Mini altitude plot
axAlt = axes('Parent',fig,'Position',[0.74 0.38 0.24 0.22]);
plot(axAlt, T, S(:,3), 'Color',[0.3 0.6 1],'LineWidth',1.2); hold(axAlt,'on');
h_altmark = plot(axAlt, T(1), S(1,3), 'wo','MarkerFaceColor','r','MarkerSize',6);
set(axAlt,'Color',[0.05 0.07 0.10],'XColor','w','YColor','w'); grid(axAlt,'on');
title(axAlt,'Altitude vs Time','Color','w','FontSize',9);

% Mini speed/mach plot
axSpd = axes('Parent',fig,'Position',[0.74 0.06 0.24 0.22]);
plot(axSpd, T, Mach_hist, 'Color',[1 0.7 0.2],'LineWidth',1.2); hold(axSpd,'on');
h_spdmark = plot(axSpd, T(1), Mach_hist(1), 'wo','MarkerFaceColor','r','MarkerSize',6);
set(axSpd,'Color',[0.05 0.07 0.10],'XColor','w','YColor','w'); grid(axSpd,'on');
title(axSpd,'Mach Number vs Time','Color','w','FontSize',9);

%% ------------------------- 5. VIDEO WRITER (OPTIONAL) ----------------------
if anim.export_video
    vw = VideoWriter(anim.video_filename, 'MPEG-4');
    vw.FrameRate = anim.playback_fps;
    open(vw);
end

%% ------------------------- 6. MAIN ANIMATION LOOP --------------------------
trail_window_samples = round(anim.trail_length_s * anim.playback_fps / anim.speedup);
particle_ptr = 1;

for i = 1:Nf
    p = pos_play(i,:);
    v = vel_play(i,:);
    vhat = v / max(norm(v), 1e-6);

    % --- Update missile body orientation (simple cone/arrow glyph) ---
    L = anim.missile_length_m; Wd = anim.missile_width_m;
    % build an orthonormal frame around velocity direction
    up_ref = [0 0 1];
    if abs(dot(vhat,up_ref)) > 0.99
        up_ref = [0 1 0];
    end
    side = cross(vhat, up_ref); side = side/max(norm(side),1e-6);
    up   = cross(side, vhat);   up = up/max(norm(up),1e-6);

    nose = p + vhat*L*0.6;
    tailC = p - vhat*L*0.4;
    tail1 = tailC + side*Wd*0.5;
    tail2 = tailC - side*Wd*0.5;
    tail3 = tailC + up*Wd*0.5;
    verts = [nose; tail1; tail3; tail2; tailC];
    set(missile_patch, 'Vertices', verts);

    set(h_shadow, 'XData', p(1), 'YData', p(2), 'ZData', 0);
    set(h_target, 'XData', tgt_play(i,1), 'YData', tgt_play(i,2), 'ZData', tgt_play(i,3));
    addpoints(h_target_trail, tgt_play(i,1), tgt_play(i,2), tgt_play(i,3));

    % --- Trail (last N samples), colored by speed ---
    i0 = max(1, i-trail_window_samples);
    xs = pos_play(i0:i,1); ys = pos_play(i0:i,2); zs = pos_play(i0:i,3);
    cs = speed_play(i0:i);
    if numel(xs) >= 2
        set(h_trail, 'XData', [xs xs], 'YData',[ys ys], 'ZData',[zs zs], ...
            'CData',[cs(:) cs(:)]);
    end

    % --- Plume particle emission while thrusting ---
    if thrust_on_play(i) && mod(i, anim.plume_emit_every) == 0
        emit_pt = p - vhat*L*0.45 + 0.15*L*(rand(1,3)-0.5);
        plume_pos(particle_ptr,:) = emit_pt;
        plume_age(particle_ptr) = 0;
        particle_ptr = mod(particle_ptr, max_particles) + 1;
    end
    % age & cull particles
    valid = ~isnan(plume_pos(:,1));
    plume_age(valid) = plume_age(valid) + playback_dt;
    expired = plume_age > anim.plume_lifetime_s;
    plume_pos(expired,:) = NaN;
    plume_age(expired) = inf;

    show = ~isnan(plume_pos(:,1));
    if any(show)
        alpha_vals = max(0, 1 - plume_age(show)/anim.plume_lifetime_s);
        sizes = 10 + 40*alpha_vals;
        set(h_plume, 'XData', plume_pos(show,1), 'YData', plume_pos(show,2), ...
            'ZData', plume_pos(show,3), 'SizeData', sizes, ...
            'CData', repmat([1 0.55 0.1], sum(show), 1), ...
            'MarkerFaceAlpha','flat', 'AlphaData', alpha_vals, ...
            'AlphaDataMapping','none');
    else
        set(h_plume, 'XData', nan, 'YData', nan, 'ZData', nan);
    end

    % --- Dynamic chase camera: orbit slightly behind & above missile ---
    az = atan2d(vhat(2), vhat(1)) - 200;
    el = 15 + 10*sin(i/60);
    view(ax3d, az, el);

    % --- HUD text ---
    hud_str = sprintf([ ...
        'TIME       : %6.2f s\n' ...
        'ALTITUDE   : %7.1f m\n' ...
        'SPEED      : %7.1f m/s\n' ...
        'MACH       : %6.2f\n' ...
        'RANGE2TGT  : %7.1f m\n' ...
        'GUID ACCEL : %6.2f g\n' ...
        'STATUS     : %s'], ...
        t_play(i), p(3), norm(v), mach_play(i), rng_play(i), accel_play(i), ...
        ternary(thrust_on_play(i), 'POWERED FLIGHT', 'COAST/TERMINAL'));
    set(hud, 'String', hud_str);

    % --- Mini plot cursors ---
    set(h_altmark, 'XData', t_play(i), 'YData', p(3));
    set(h_spdmark, 'XData', t_play(i), 'YData', mach_play(i));

    drawnow limitrate;

    if anim.export_video
        frame = getframe(fig);
        writeVideo(vw, frame);
    end
end

if anim.export_video
    close(vw);
    fprintf('Video saved to: %s\n', anim.video_filename);
end

%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function [ds, diag_out] = missile_dynamics(t, s, cfg, target_pos)
    pos = s(1:3); vel = s(4:6); m = s(7); alt = pos(3);
    [rho, a_sound] = std_atmosphere(alt);

    wind = cfg.wind_NED(:)'; wind(3) = -wind(3);
    if cfg.use_gusts
        wind = wind + cfg.gust_std_ms*randn(1,3);
    end
    v_rel = vel(:)' - wind;
    V = max(norm(v_rel), 1e-6);
    mach = V / a_sound;

    Cd = interp1(cfg.mach_table, cfg.cd_table, mach, 'linear', 'extrap');
    q = 0.5*rho*V^2;
    F_drag = -(q*Cd*cfg.ref_area_m2) * (v_rel/V)';

    thrust_mag = 0;
    tp = cfg.thrust_profile;
    for i = 1:size(tp,1)
        if t >= tp(i,1) && t < tp(i,2)
            thrust_mag = tp(i,3); break;
        end
    end
    speed_dir = vel(:)/max(norm(vel),1e-6);

    accel_cmd_g = 0; F_guidance = [0;0;0];
    if cfg.guidance_on && t >= cfg.guidance_start_t
        r_vec = target_pos(:) - pos(:);
        r_mag = norm(r_vec);
        if r_mag > 1e-3
            v_target_rel = cfg.target_vel_ms(:) - vel(:);
            los_rate_vec = cross(r_vec, v_target_rel) / (r_mag^2);
            Vc = -dot(r_vec, v_target_rel)/r_mag;
            v_dir = vel(:)/max(norm(vel),1e-6);
            a_cmd_vec = cfg.N_gain * Vc * cross(los_rate_vec, v_dir);
            a_cmd_mag = norm(a_cmd_vec);
            max_a = cfg.max_lateral_accel_g * cfg.g0;
            if a_cmd_mag > max_a
                a_cmd_vec = a_cmd_vec * (max_a/a_cmd_mag);
                a_cmd_mag = max_a;
            end
            accel_cmd_g = a_cmd_mag / cfg.g0;
            F_guidance = m * a_cmd_vec;
        end
    end

    g_alt = cfg.g0 * (cfg.Re/(cfg.Re+max(alt,0)))^2;
    F_gravity = [0;0;-m*g_alt];
    F_thrust = thrust_mag * speed_dir;

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
    ds(1:3) = vel; ds(4:6) = accel; ds(7) = mdot;
    diag_out.mach = mach; diag_out.accel_cmd_g = accel_cmd_g;
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
