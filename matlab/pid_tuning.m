% pid_tuning.m - offline model of one servo axis with the controller's discrete PID.
% Plant: J*dw/dt = u - b*w - Fc*sign(w), the same model as src/servo_sim.cpp.
% Edit the gains, check the step response, then copy them into src/pid.hpp.
% Plain MATLAB (no toolboxes), also runs in GNU Octave.

J = 0.01; b = 0.1; Fc = 0.05;            % inertia, viscous damping, Coulomb friction
Kp = 20; Ki = 10; Kd = 1.2;              % same defaults as PidGains in src/pid.hpp
umax = 2.0; fd = 100;                    % torque limit [Nm], derivative filter [Hz]
Ts = 1e-3; T = 1.5; sub = 10;            % 1 kHz loop, 1.5 s step, physics substeps
target = 90 * pi/180;

n = round(T/Ts); t = (0:n-1)*Ts;
pos = 0; vel = 0; integ = 0; d = 0; prev = 0;
alpha = Ts / (Ts + 1/(2*pi*fd));
P = zeros(1,n); U = zeros(1,n);
for k = 1:n
    err = target - pos;
    if k > 1, d = d + alpha*(-(pos - prev)/Ts - d); end
    prev = pos;
    unsat = Kp*err + integ + Kd*d;
    u = min(max(unsat, -umax), umax);
    if unsat == u || ((unsat > u) ~= (err > 0)), integ = integ + Ki*err*Ts; end
    integ = min(max(integ, -umax), umax);
    for s = 1:sub                         % integrate the plant between controller ticks
        h = Ts/sub;
        if abs(vel) < 1e-3 && abs(u) <= Fc
            vel = 0;
        else
            fr = Fc * sign(vel + (vel == 0)*u);
            vel = vel + h*(u - b*vel - fr)/J;
            pos = pos + h*vel;
        end
    end
    P(k) = pos; U(k) = u;
end

deg = P*180/pi;
overshoot = max(deg) - 90;
settle = t(find(abs(deg - 90) > 1, 1, 'last') + 1);
if isempty(settle), settle = 0; end
fprintf('Step 0 -> 90 deg: overshoot %.2f deg, settling time (1 deg) %.3f s, final error %.3f deg\n', ...
        overshoot, settle, deg(end) - 90);

subplot(2,1,1); plot(t, deg, t, 90*ones(size(t)), '--'); grid on;
ylabel('position [deg]'); title(sprintf('Kp=%g Ki=%g Kd=%g', Kp, Ki, Kd));
subplot(2,1,2); plot(t, U); grid on; ylabel('torque [Nm]'); xlabel('time [s]');
