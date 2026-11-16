%% tune_pi.m  -  PI tuning for the hybrid-storage controller (pidtune method)
% Needs MATLAB + Control System Toolbox. Check with:  ver control
% Run the whole file (Run button) or one %% section at a time (Ctrl+Enter).

clear; close all;

% Everything this script prints and plots is also saved to a 'results' folder
% next to it (see section 5), ready to upload to GitHub.
outdir = 'results';
if ~exist(outdir, 'dir'), mkdir(outdir); end
logfile = fullfile(outdir, 'command_window.txt');
if exist(logfile, 'file'), delete(logfile); end
diary(logfile);

%% Plant values - taken from the Typhoon model (buck.test.tse)
Vbus = 48;          % bus voltage [V]
L    = 1e-3;        % inductor L1 / L2 [H]
C    = 3300e-6;     % bus capacitor C3 [F]
D    = 0.5;         % duty cycle with 24 V storage on a 48 V bus
Td   = 75e-6;       % C2000 delay: about 1.5 switching periods at 20 kHz [s]

opts = pidtuneOptions('PhaseMargin', 60);   % stability margin to aim for [deg]

%% 1) Current loop - same gains for PI_Controller_I (battery) and PI_I1 (supercap)
% Plant: duty cycle -> inductor current. 48 V across 1 mH per unit of duty,
% plus the delay the real C2000 will add.
Gi = tf(Vbus, [L 0], 'InputDelay', Td);
Ci = pidtune(Gi, 'PI', 2*pi*1000, opts);    % aim for about 1 kHz

[~, PMi, ~, wci] = margin(Ci*Gi);
fprintf('\nCURRENT LOOP   Kp = %.3f   Ki = %.1f   (speed %.0f Hz, margin %.0f deg)\n', ...
        Ci.Kp, Ci.Ki, wci/(2*pi), PMi);
fprintf('   model now:  Kp = 0.200   Ki = 192.0\n');

figI = figure('Name','Current loop - open loop'); margin(Ci*Gi);   % title shows Gm and Pm
figIstep = figure('Name','Current loop - step'); step(feedback(Ci*Gi, 1), 2e-3); grid on;
title('Current loop - response to a 1 A step in the current target');

%% 2) Voltage loop - PI_Controller_V
% The voltage loop sees the bus capacitor THROUGH the closed current loop.
% pade() swaps the delay for an equivalent fraction so pidtune can handle it.
Ti = feedback(Ci*pade(Gi, 3), 1);           % closed current loop
Gv = Ti * tf(D, [C 0]);                     % current target -> bus voltage
Cv = pidtune(Gv, 'PI', 2*pi*100, opts);     % aim for about 100 Hz

[~, PMv, ~, wcv] = margin(Cv*Gv);
fprintf('VOLTAGE LOOP   Kp = %.3f   Ki = %.1f   (speed %.0f Hz, margin %.0f deg)\n', ...
        Cv.Kp, Cv.Ki, wcv/(2*pi), PMv);
fprintf('   model now:  Kp = 4.200   Ki = 265.0\n');

figV = figure('Name','Voltage loop - open loop'); margin(Cv*Gv);   % title shows Gm and Pm

%% 3) Check - how far does the bus dip on a 1.2 A load step?
% Load current pulls on the capacitor; the voltage loop pushes back.
Gd = feedback(tf(1, [C 0]), D*Ti*Cv);
[y, t] = step(-1.2*Gd, 0.05);
fprintf('BUS DIP for a 1.2 A load step: %.2f V\n\n', -min(y));
figBus = figure('Name','Bus dip'); plot(t*1e3, 48 + y, 'LineWidth', 1.5); grid on;
xlabel('time [ms]'); ylabel('bus voltage [V]');
title('Bus voltage after a 1.2 A load step (compare with Typhoon)');

%% 4) Grid-side controller PI_grid - only once you build it
% The rectifier alone holding the bus: source current -> bus voltage through
% C3 and the average load (48 V / 1.8 A = 26.7 ohm), seen through the
% 10 Hz filter on its voltage measurement.
% Margin target is 45 deg here, not 60: at 10 Hz the 10 Hz filter alone costs
% 45 deg and the capacitor about 80 deg, so a PI cannot reach 60 deg.
R  = 26.7;
Gg = tf(1, [C 1/R]) * tf(1, [1/(2*pi*10) 1]);
optsG = pidtuneOptions('PhaseMargin', 45);
Cg = pidtune(Gg, 'PI', 2*pi*10, optsG);     % aim for about 10 Hz

[~, PMg, ~, wcg] = margin(Cg*Gg);
fprintf('GRID LOOP      Kp = %.3f   Ki = %.2f   (speed %.1f Hz, margin %.0f deg)\n', ...
        Cg.Kp, Cg.Ki, wcg/(2*pi), PMg);
fprintf('   planned:    Kp = 0.200   Ki = 2.50\n\n');

%% 5) Save results - gains table, plots, and the Command Window text
T = table(["Current"; "Voltage"; "Grid"], ...
          [Ci.Kp; Cv.Kp; Cg.Kp], [Ci.Ki; Cv.Ki; Cg.Ki], ...
          [wci; wcv; wcg]/(2*pi), [PMi; PMv; PMg], ...
          'VariableNames', {'Loop', 'Kp', 'Ki', 'Speed_Hz', 'PhaseMargin_deg'});
writetable(T, fullfile(outdir, 'pi_gains.csv'));

print(figI,     fullfile(outdir, 'current_loop_bode.png'), '-dpng', '-r200');
print(figIstep, fullfile(outdir, 'current_loop_step.png'), '-dpng', '-r200');
print(figV,     fullfile(outdir, 'voltage_loop_bode.png'), '-dpng', '-r200');
print(figBus,   fullfile(outdir, 'bus_dip.png'),           '-dpng', '-r200');

fprintf('Saved to %s: pi_gains.csv, 4 plots, command_window.txt\n', fullfile(pwd, outdir));
diary off;
