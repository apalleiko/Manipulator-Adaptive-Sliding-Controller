function [] = animate_traj(robot,q,rate)
% Animate trajectory
count = size(q,2);
figure
show(robot,q(:,1));
xlim([-.01,.1])
ylim([-2.5,2.5])
zlim([-2.5,2.5])
view(90,0)
while true
    for i = 1:count
        show(robot,q(:,i),'PreservePlot',false);
        xlim([-.01,.1])
        view(90,0)
        drawnow
        pause(rate);
    end
    pause(0.5);
end
end

