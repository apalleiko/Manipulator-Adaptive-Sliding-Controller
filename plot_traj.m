function [] = plot_traj(t,x,n,label)
%PLOT_TRAJ Summary of this function goes here
%   Detailed explanation goes here
figure
for i=1:n
    subplot(n,1,i)
    plot(t,x(:,i))
    if i==1
        title(label)
    end
    xlabel("Time (s)")
    ylabel(['q_',int2str(i),' (rad)'])
end

