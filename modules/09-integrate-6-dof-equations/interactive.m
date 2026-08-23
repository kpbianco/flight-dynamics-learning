function interactive
%INTERACTIVE Explore force and moment inputs to the P09 6-DOF integrator.
clear model;
modelFcn=@model;
existingUi=findall(groot,'Type','figure','Name','P09 6-DOF Integrator');
if ~isempty(existingUi)
    close(existingUi);
end
fig=uifigure('Name','P09 6-DOF Integrator','Position',[20 60 1800 820]);
gridLayout=uigridlayout(fig,[6 6]);
gridLayout.RowHeight={'1x','1x','1x',126,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x','1x','1x'};

axPath=uiaxes(gridLayout);
axPath.Layout.Row=[1 3]; axPath.Layout.Column=1;
axDown=uiaxes(gridLayout);
axDown.Layout.Row=[1 3]; axDown.Layout.Column=2;
axSpeed=uiaxes(gridLayout);
axSpeed.Layout.Row=[1 3]; axSpeed.Layout.Column=3;
axRates=uiaxes(gridLayout);
axRates.Layout.Row=[1 3]; axRates.Layout.Column=4;
axAttitude=uiaxes(gridLayout);
axAttitude.Layout.Row=[1 3]; axAttitude.Layout.Column=5;
axBroken=uiaxes(gridLayout);
axBroken.Layout.Row=[1 3]; axBroken.Layout.Column=6;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 6];

forceLabel=uilabel(gridLayout, ...
    'Text','Forward-force half-sine pulse scale (-)', ...
    'HorizontalAlignment','center');
forceLabel.Layout.Row=5; forceLabel.Layout.Column=[1 3];
momentLabel=uilabel(gridLayout, ...
    'Text','Three-axis moment half-sine pulse scale (-)', ...
    'HorizontalAlignment','center');
momentLabel.Layout.Row=5; momentLabel.Layout.Column=[4 6];

forceControl=uislider(gridLayout,'Limits',[0 1.5],'Value',1, ...
    'MajorTicks',[0 0.5 1 1.5]);
forceControl.Layout.Row=6; forceControl.Layout.Column=[1 3];
momentControl=uislider(gridLayout,'Limits',[0 1.5],'Value',1, ...
    'MajorTicks',[0 0.5 1 1.5]);
momentControl.Layout.Row=6; momentControl.Layout.Column=[4 6];

forceControl.ValueChangingFcn=@(~,event) updatePlots(event,'force');
momentControl.ValueChangingFcn=@(~,event) updatePlots(event,'moment');
controls=[forceControl momentControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        forcePulseScale=forceControl.Value;
        momentPulseScale=momentControl.Value;
        if nargin==2
            switch changingControl
                case 'force'
                    forcePulseScale=event.Value;
                case 'moment'
                    momentPulseScale=event.Value;
            end
        end
        out=modelFcn(forcePulseScale,momentPulseScale);

        cla(axPath);
        plot(axPath,out.north_m,out.east_m,'LineWidth',1.6);
        hold(axPath,'on');
        plot(axPath,out.north_m(1),out.east_m(1),'ko', ...
            'MarkerFaceColor','k');
        grid(axPath,'on'); axis(axPath,'equal');
        xlabel(axPath,'North (m)'); ylabel(axPath,'East (m)');
        title(axPath,'NED horizontal path');

        cla(axDown);
        plot(axDown,out.time_s,out.down_m,'LineWidth',1.6);
        hold(axDown,'on');
        plot(axDown,out.time_s,zeros(size(out.time_s)),'k--');
        grid(axDown,'on');
        xlabel(axDown,'Time (s)'); ylabel(axDown,'Down (m)');
        title(axDown,'Down-positive position');

        cla(axSpeed);
        plot(axSpeed,out.time_s,out.speed_mps,'LineWidth',1.6);
        grid(axSpeed,'on');
        xlabel(axSpeed,'Time (s)'); ylabel(axSpeed,'Inertial speed (m/s)');
        title(axSpeed,'Force-to-velocity response');

        cla(axRates);
        plot(axRates,out.time_s,out.bodyRates_deg_s,'LineWidth',1.3);
        grid(axRates,'on');
        xlabel(axRates,'Time (s)'); ylabel(axRates,'Body rate (deg/s)');
        legend(axRates,{'p','q','r'},'Location','best');
        title(axRates,'Moment-to-rate response');

        cla(axAttitude);
        plot(axAttitude,out.time_s,out.eulerAngles_deg,'LineWidth',1.3);
        grid(axAttitude,'on');
        xlabel(axAttitude,'Time (s)');
        ylabel(axAttitude,'Derived attitude (deg)');
        legend(axAttitude,{'roll','pitch','yaw'},'Location','best');
        title(axAttitude,'Quaternion-derived display');

        cla(axBroken);
        plot(axBroken,out.time_s,out.brokenPositionError_m, ...
            'LineWidth',1.6); hold(axBroken,'on');
        plot(axBroken,out.time_s, ...
            out.brokenTransportResidualMagnitude_mps2,'--','LineWidth',1.4);
        grid(axBroken,'on');
        xlabel(axBroken,'Time (s)');
        ylabel(axBroken,'Error (m) or residual (m/s^2)');
        legend(axBroken,{'position separation','closure residual'}, ...
            'Location','best');
        title(axBroken,'Omit -omega cross v');

        summary.Text=sprintf([ ...
            'Move one lever and reset: F scale %.2f changes only forward body force; M scale %.2f changes only the three-axis moment pulse.\n' ...
            'final N/E/D = %.1f / %.1f / %.1f m | final speed %.1f m/s | peak rate %.1f deg/s | peak attitude rotation %.1f deg\n' ...
            'q_NB norm error %.2g | C_NB orthogonality error %.2g | post-pulse inertial H drift %.2g | broken final path error %.1f m'], ...
            out.forcePulseScale,out.momentPulseScale, ...
            out.finalPositionNED_m(1),out.finalPositionNED_m(2), ...
            out.finalPositionNED_m(3),out.finalSpeed_mps, ...
            out.peakBodyRateMagnitude_deg_s,out.peakAttitudeRotation_deg, ...
            max(abs(out.quaternionNorm-1)), ...
            max(out.dcmOrthonormalityError), ...
            out.postPulseAngularMomentumRelativeDrift, ...
            out.brokenFinalPositionError_m);
    end
end
