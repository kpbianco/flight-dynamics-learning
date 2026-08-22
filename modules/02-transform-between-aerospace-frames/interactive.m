function interactive
%INTERACTIVE Explore one frame-transform lever at a time.
clear model;
modelFcn=@model;
fig=uifigure('Name','P02 Aerospace Frame Transform','Position',[80 80 1240 760]);
gridLayout=uigridlayout(fig,[6 6]);
gridLayout.RowHeight={'1x','1x','1x',72,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x','1x','1x'};

axBody=uiaxes(gridLayout);
axBody.Layout.Row=[1 3]; axBody.Layout.Column=[1 2];
axNed=uiaxes(gridLayout);
axNed.Layout.Row=[1 3]; axNed.Layout.Column=[3 4];
axComponents=uiaxes(gridLayout);
axComponents.Layout.Row=[1 3]; axComponents.Layout.Column=[5 6];

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 6];

speedLabel=uilabel(gridLayout,'Text','Speed V (m/s)','HorizontalAlignment','center');
speedLabel.Layout.Row=5; speedLabel.Layout.Column=1;
alphaLabel=uilabel(gridLayout,'Text','Angle of attack alpha (deg)','HorizontalAlignment','center');
alphaLabel.Layout.Row=5; alphaLabel.Layout.Column=2;
betaLabel=uilabel(gridLayout,'Text','Sideslip beta (deg)','HorizontalAlignment','center');
betaLabel.Layout.Row=5; betaLabel.Layout.Column=3;
rollLabel=uilabel(gridLayout,'Text','Roll phi (deg)','HorizontalAlignment','center');
rollLabel.Layout.Row=5; rollLabel.Layout.Column=4;
pitchLabel=uilabel(gridLayout,'Text','Pitch theta (deg)','HorizontalAlignment','center');
pitchLabel.Layout.Row=5; pitchLabel.Layout.Column=5;
yawLabel=uilabel(gridLayout,'Text','Yaw psi (deg)','HorizontalAlignment','center');
yawLabel.Layout.Row=5; yawLabel.Layout.Column=6;

speedControl=uislider(gridLayout,'Limits',[20 200],'Value',70, ...
    'MajorTicks',[20 50 100 150 200]);
speedControl.Layout.Row=6; speedControl.Layout.Column=1;
alphaControl=uislider(gridLayout,'Limits',[-15 20],'Value',6, ...
    'MajorTicks',[-15 0 10 20]);
alphaControl.Layout.Row=6; alphaControl.Layout.Column=2;
betaControl=uislider(gridLayout,'Limits',[-20 20],'Value',0, ...
    'MajorTicks',[-20 -10 0 10 20]);
betaControl.Layout.Row=6; betaControl.Layout.Column=3;
rollControl=uislider(gridLayout,'Limits',[-60 60],'Value',0, ...
    'MajorTicks',[-60 -30 0 30 60]);
rollControl.Layout.Row=6; rollControl.Layout.Column=4;
pitchControl=uislider(gridLayout,'Limits',[-45 45],'Value',9, ...
    'MajorTicks',[-45 -20 0 20 45]);
pitchControl.Layout.Row=6; pitchControl.Layout.Column=5;
yawControl=uislider(gridLayout,'Limits',[-180 180],'Value',30, ...
    'MajorTicks',[-180 -90 0 90 180]);
yawControl.Layout.Row=6; yawControl.Layout.Column=6;

speedControl.ValueChangingFcn=@(~,event) updatePlots(event,'speed');
alphaControl.ValueChangingFcn=@(~,event) updatePlots(event,'alpha');
betaControl.ValueChangingFcn=@(~,event) updatePlots(event,'beta');
rollControl.ValueChangingFcn=@(~,event) updatePlots(event,'roll');
pitchControl.ValueChangingFcn=@(~,event) updatePlots(event,'pitch');
yawControl.ValueChangingFcn=@(~,event) updatePlots(event,'yaw');
controls=[speedControl alphaControl betaControl rollControl pitchControl yawControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        speed=speedControl.Value;
        alpha=alphaControl.Value;
        beta=betaControl.Value;
        roll=rollControl.Value;
        pitch=pitchControl.Value;
        yaw=yawControl.Value;
        if nargin==2
            switch changingControl
                case 'speed'
                    speed=event.Value;
                case 'alpha'
                    alpha=event.Value;
                case 'beta'
                    beta=event.Value;
                case 'roll'
                    roll=event.Value;
                case 'pitch'
                    pitch=event.Value;
                case 'yaw'
                    yaw=event.Value;
            end
        end

        out=modelFcn(speed,alpha,beta,roll,pitch,yaw);
        plotLimit=1.1*out.speed_mps;

        cla(axBody);
        quiver3(axBody,0,0,0,out.velocityBody_mps(1),out.velocityBody_mps(2), ...
            out.velocityBody_mps(3),0,'LineWidth',2,'MaxHeadSize',0.25);
        grid(axBody,'on'); axis(axBody,'equal'); view(axBody,35,24);
        xlim(axBody,[-plotLimit plotLimit]); ylim(axBody,[-plotLimit plotLimit]);
        zlim(axBody,[-plotLimit plotLimit]); axBody.ZDir='reverse';
        xlabel(axBody,'x_b forward (m/s)'); ylabel(axBody,'y_b right (m/s)');
        zlabel(axBody,'z_b down (m/s)'); title(axBody,'Body components [u v w]');

        cla(axNed);
        quiver3(axNed,0,0,0,out.velocityNed_mps(1),out.velocityNed_mps(2), ...
            out.velocityNed_mps(3),0,'LineWidth',2,'MaxHeadSize',0.25);
        grid(axNed,'on'); axis(axNed,'equal'); view(axNed,35,24);
        xlim(axNed,[-plotLimit plotLimit]); ylim(axNed,[-plotLimit plotLimit]);
        zlim(axNed,[-plotLimit plotLimit]); axNed.ZDir='reverse';
        xlabel(axNed,'North (m/s)'); ylabel(axNed,'East (m/s)');
        zlabel(axNed,'Down (m/s)'); title(axNed,'NED components [N E D]');

        cla(axComponents);
        bar(axComponents,[out.velocityBody_mps out.velocityNed_mps]);
        grid(axComponents,'on');
        axComponents.XTick=1:3;
        axComponents.XTickLabel={'axis 1','axis 2','axis 3'};
        ylabel(axComponents,'Velocity component (m/s)');
        title(axComponents,'Coordinates change; vector norm does not');
        legend(axComponents,{'body: u/v/w','NED: N/E/D'},'Location','best');

        if out.trackDefined
            trackText=sprintf('track %.2f deg',out.trackDeg);
        else
            trackText='track undefined (vertical vector)';
        end
        summary.Text=sprintf([ ...
            'Move one lever, then reset it before moving another.  %s | flight path %.2f deg (positive climb)\n' ...
            'body [u v w] = [%.2f %.2f %.2f] m/s | NED [N E D] = [%.2f %.2f %.2f] m/s\n' ...
            'round-trip error %.3g m/s | norm error %.3g m/s'], ...
            trackText,out.flightPathDeg, ...
            out.velocityBody_mps(1),out.velocityBody_mps(2),out.velocityBody_mps(3), ...
            out.velocityNed_mps(1),out.velocityNed_mps(2),out.velocityNed_mps(3), ...
            out.roundTripError_mps,out.normError_mps);
    end
end
