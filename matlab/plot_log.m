% plot_log.m - plot a run recorded with: motion_controller --demo --log run.csv
% Columns: t, then sp<i>, pos<i>, effort<i> for each axis.
function plot_log(file)
    if nargin < 1, file = 'run.csv'; end
    m = dlmread(file, ',', 1, 0);
    t = m(:,1);
    axes_n = (size(m,2) - 1) / 3;
    for i = 1:axes_n
        c = 2 + (i-1)*3;
        subplot(axes_n, 1, i);
        plot(t, m(:,c), '--', t, m(:,c+1)); grid on;
        ylabel(sprintf('axis %d [deg]', i));
        rms_err = sqrt(mean((m(:,c) - m(:,c+1)).^2));
        title(sprintf('axis %d  RMS tracking error %.2f deg', i, rms_err));
    end
    xlabel('time [s]'); legend('setpoint', 'position');
end
