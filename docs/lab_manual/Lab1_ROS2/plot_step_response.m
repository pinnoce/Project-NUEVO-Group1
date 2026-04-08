% plot_step_response.m
% DC Motor Velocity Step Response Analysis
%
% Loads a CSV exported from the NUEVO UI and plots the closed-loop
% velocity step response with all metrics required by the lab manual:
%   - Rise time  t_r  : 10 % to 90 % of setpoint
%   - Overshoot  M_p  : (peak - setpoint) / setpoint * 100 %
%   - Settling time t_s : last sample outside the ±5 % band
%   - Steady-state error e_ss : setpoint - mean(final velocity)
%
% Usage:
%   Place this file in the same folder as the CSV and run it.
%   Edit CSV_FILE and SETPOINT if needed.

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────────────────
CSV_FILE = 'dc_motor_1_0331_15-28.csv';
SETPOINT = 1000;    % velocity setpoint (ticks/s)
BAND_PCT = 5;       % settling band width (%)
PLOT_TIME = 2.0;    % seconds
FONT_SIZE = 15;     % common plot font size

%% ── Colour palette (material-design inspired) ────────────────────────────
C_VEL      = [0.102 0.463 0.824];   % blue  700   — velocity trace
C_SP       = [0.263 0.345 0.384];   % blue-grey 700 — setpoint line
C_BAND     = [1.000 0.976 0.882];   % amber 50   — band fill
C_RISE     = [0.216 0.553 0.235];   % green 700  — rise-time markers
C_PEAK     = [0.827 0.184 0.184];   % red   700  — peak marker
C_SETTLE   = [0.482 0.106 0.639];   % purple 700 — settling line
C_BOX_BG   = [0.976 0.976 0.980];   % near-white box background
C_BOX_EDGE = [0.400 0.400 0.420];   % medium grey box border

%% ── Load data ────────────────────────────────────────────────────────────
raw      = readmatrix(CSV_FILE, 'NumHeaderLines', 1);
time_raw = raw(:, 2);
vel_raw  = raw(:, 4);
pwm_raw  = raw(:, 6);

%% ── Isolate step region ──────────────────────────────────────────────────
step_idx = find(pwm_raw ~= 0, 1, 'first');

if isempty(step_idx)
    error('No non-zero PWM found — check the CSV file.');
end

step_start_time = time_raw(step_idx);
step_end_idx = find(time_raw >= step_start_time + PLOT_TIME, 1, 'first');
if isempty(step_end_idx)
    step_end_idx = length(time_raw);
end

t   = time_raw(step_idx:step_end_idx) - step_start_time;
vel = vel_raw(step_idx:step_end_idx);

%% ── Performance metrics ──────────────────────────────────────────────────
band = BAND_PCT / 100 * SETPOINT;

% Steady-state: mean of the final 30 % of samples
v_ss = mean(vel(round(0.70 * length(vel)):end));

% Rise time: first crossing of 10 % and 90 % of setpoint
v10    = 0.10 * SETPOINT;
v90    = 0.90 * SETPOINT;
idx_10 = find(vel >= v10, 1, 'first');
idx_90 = find(vel >= v90, 1, 'first');
if isempty(idx_10) || isempty(idx_90)
    error('Velocity never reached 10 %% or 90 %% of setpoint.');
end
t_r = t(idx_90) - t(idx_10);

% Peak overshoot
[v_peak, peak_idx] = max(vel);
Mp     = (v_peak - SETPOINT) / SETPOINT * 100;
t_peak = t(peak_idx);

% Settling time: last sample outside the ±band
out_idx = find(vel < SETPOINT - band | vel > SETPOINT + band);
if isempty(out_idx)
    t_s = 0;
else
    t_s = t(out_idx(end));
end

% Steady-state error
e_ss = SETPOINT - v_ss;

%% ── Print summary ────────────────────────────────────────────────────────
fprintf('\n=== Step Response Metrics ===\n');
fprintf('  Rise time       t_r  = %.3f s   (10 %%–90 %% of setpoint)\n', t_r);
fprintf('  Overshoot       M_p  = %.1f %%\n', Mp);
fprintf('  Settling time   t_s  = %.3f s   (±%d %% band)\n', t_s, BAND_PCT);
fprintf('  Steady-state    v_ss = %.1f ticks/s\n', v_ss);
fprintf('  SS error        e_ss = %.1f ticks/s\n', e_ss);

%% ── Plot ─────────────────────────────────────────────────────────────────
figure('Position', [100 100 980 540], 'Color', 'w');
ax = axes('FontSize', FONT_SIZE, 'LineWidth', 0.8);
hold on;

% ±5 % settling band
fill([t(1); t(end); t(end); t(1)], ...
     [SETPOINT-band; SETPOINT-band; SETPOINT+band; SETPOINT+band], ...
     C_BAND, 'EdgeColor', C_BAND * 0.88, 'LineWidth', 0.8, ...
     'FaceAlpha', 0.85, ...
     'DisplayName', sprintf('±%d%% settling band', BAND_PCT));

% Measured velocity
plot(t, vel, '-', 'Color', C_VEL, 'LineWidth', 2.2, ...
     'DisplayName', 'Measured velocity');

% Setpoint
yline(SETPOINT, '--', 'LineWidth', 1.6, 'Color', C_SP, ...
      'DisplayName', sprintf('Setpoint (%d t/s)', SETPOINT));

% Rise time: markers at 10 % and 90 % crossings
plot([t(idx_10) t(idx_90)], [v10 v90], 'o-', ...
     'Color', C_RISE, 'LineWidth', 2.0, ...
     'MarkerSize', 8, 'MarkerFaceColor', C_RISE, ...
     'DisplayName', sprintf('Rise time  t_r = %.3f s', t_r));

% Peak overshoot marker
plot(t_peak, v_peak, 'v', ...
     'Color', C_PEAK, 'MarkerSize', 11, 'MarkerFaceColor', C_PEAK, ...
     'LineWidth', 1.5, ...
     'DisplayName', sprintf('Peak  M_p = %.1f %%', Mp));

% Settling time vertical line
xline(t_s, '--', 'LineWidth', 1.6, 'Color', C_SETTLE, ...
      'DisplayName', sprintf('Settling time  t_s = %.3f s', t_s));

%% ── Metrics text box (axes text — auto-sizes, no overflow) ───────────────
metrics = sprintf(['t_r  =  %.3f s   (10%% to 90%% of SP)\n' ...
                   'M_p  =  %.1f %%\n' ...
                   't_s  =  %.3f s   (+/-%d%% band)\n' ...
                   'e_ss =  %.1f t/s'], ...
                   t_r, Mp, t_s, BAND_PCT, e_ss);

text(0.55, 0.05, metrics, ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'bottom', ...
    'BackgroundColor', C_BOX_BG, ...
    'EdgeColor', C_BOX_EDGE, ...
    'FontName', 'Courier New', ...
    'FontSize', FONT_SIZE, ...
    'Margin', 10, ...
    'LineWidth', 0.8);

%% ── Axes formatting ──────────────────────────────────────────────────────
xlabel('Time (s)',       'FontSize', 20);
ylabel('Velocity (t/s)', 'FontSize', 20);
title('DC Motor 1 — Velocity Step Response  (0 \rightarrow 1000 t/s)', ...
      'FontSize', 20, 'Interpreter', 'tex');

leg = legend('Location', 'northeast', 'FontSize', FONT_SIZE, 'Box', 'on');
leg.BoxFace.ColorType = 'truecoloralpha';
leg.BoxFace.ColorData = uint8([248 248 250 230]');

grid on;  box on;
xlim([0 min(PLOT_TIME, t(end))]);
ylim([-50  v_peak * 1.15]);
ax.GridAlpha = 0.15;
ax.GridColor = [0.3 0.3 0.3];

%% ── Save figure ──────────────────────────────────────────────────────────
[~, base, ~] = fileparts(CSV_FILE);
out_file = [base '_step_response.png'];
exportgraphics(gcf, out_file, 'Resolution', 150);
fprintf('\nFigure saved to: %s\n', out_file);
