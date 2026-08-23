function interactive
%INTERACTIVE Explore P12 specific-force and heading convention levers.
% Move one control, inspect one transition, then reset before moving the next.
clear model;
modelFcn=@model;
uiName='P12 Energy and Frame Convention Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[90 70 1220 840]);
layout=uigridlayout(fig,[5 2]);
layout.RowHeight={72,72,36,'1x',102};
layout.ColumnWidth={'1x','1x'};

forcePanel=uipanel(layout, ...
    'Title','Lever 1 - forward non-gravity specific force');
forcePanel.Layout.Row=1;
forcePanel.Layout.Column=[1 2];
forceGrid=uigridlayout(forcePanel,[2 2]);
forceGrid.RowHeight={22,'1x'};
forceGrid.ColumnWidth={280,'1x'};
forceLabel=uilabel(forceGrid,'Text','Body-x specific force (m/s^2)');
forceLabel.Layout.Row=1;
forceLabel.Layout.Column=1;
forceValue=uilabel(forceGrid,'Text','1.50 m/s^2');
forceValue.Layout.Row=2;
forceValue.Layout.Column=1;
forceControl=uislider(forceGrid,'Limits',[0 3],'Value',1.5, ...
    'MajorTicks',[0 0.75 1.5 2.25 3]);
forceControl.Layout.Row=[1 2];
forceControl.Layout.Column=2;

headingPanel=uipanel(layout,'Title','Lever 2 - NED heading');
headingPanel.Layout.Row=2;
headingPanel.Layout.Column=[1 2];
headingGrid=uigridlayout(headingPanel,[2 2]);
headingGrid.RowHeight={22,'1x'};
headingGrid.ColumnWidth={280,'1x'};
headingLabel=uilabel(headingGrid, ...
    'Text','Heading, clockwise North to East (deg)');
headingLabel.Layout.Row=1;
headingLabel.Layout.Column=1;
headingValue=uilabel(headingGrid,'Text','30 deg');
headingValue.Layout.Row=2;
headingValue.Layout.Column=1;
headingControl=uislider(headingGrid,'Limits',[-180 180],'Value',30, ...
    'MajorTicks',[-180 -90 0 30 90 180]);
headingControl.Layout.Row=[1 2];
headingControl.Layout.Column=2;

resetControl=uibutton(layout,'push', ...
    'Text',['Reset both levers to baseline ' ...
    '(specific force 1.50 m/s^2, heading 30 deg)']);
resetControl.Layout.Row=3;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=4;
plotGrid.Layout.Column=[1 2];
axPath=uiaxes(plotGrid);
axVelocity=uiaxes(plotGrid);
axEnergy=uiaxes(plotGrid);
axBroken=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=5;
summary.Layout.Column=[1 2];

forceControl.ValueChangingFcn=@(source,event) updatePlots(event,'force');
forceControl.ValueChangedFcn=@(source,event) updatePlots(event,'force');
headingControl.ValueChangingFcn=@(source,event) updatePlots(event,'heading');
headingControl.ValueChangedFcn=@(source,event) updatePlots(event,'heading');
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        forceControl.Value=1.5;
        headingControl.Value=30;
        updatePlots();
    end

    function updatePlots(event,changingControl)
        forwardSpecificForce_mps2=forceControl.Value;
        headingAngle_deg=headingControl.Value;
        if nargin==2
            switch changingControl
                case 'force'
                    forwardSpecificForce_mps2=event.Value;
                case 'heading'
                    headingAngle_deg=event.Value;
            end
        end
        out=modelFcn(forwardSpecificForce_mps2,headingAngle_deg);
        forceValue.Text=sprintf('%.2f m/s^2', ...
            out.forwardSpecificForce_mps2);
        headingValue.Text=sprintf('%.1f deg',out.headingAngle_deg);

        cla(axPath);
        plot(axPath,out.positionNED_m(1,:),out.positionNED_m(2,:), ...
            'LineWidth',1.7);
        grid(axPath,'on'); axis(axPath,'equal');
        xlabel(axPath,'North (m)'); ylabel(axPath,'East (m)');
        title(axPath,'Active yaw rotates the path in fixed NED');

        cla(axVelocity);
        plot(axVelocity,out.time_s,out.velocityNED_mps.','LineWidth',1.3);
        grid(axVelocity,'on');
        xlabel(axVelocity,'Time (s)');
        ylabel(axVelocity,'NED velocity component (m/s)');
        legend(axVelocity,{'North','East','Down'},'Location','best');
        title(axVelocity,'Nose-up force points partly upward');

        cla(axEnergy);
        plot(axEnergy,out.time_s, ...
            (out.mechanicalEnergy_J-out.mechanicalEnergy_J(1))/1e3, ...
            'LineWidth',1.6); hold(axEnergy,'on');
        plot(axEnergy,out.time_s,out.workInput_J/1e3,'k--','LineWidth',1.3);
        grid(axEnergy,'on');
        xlabel(axEnergy,'Time (s)'); ylabel(axEnergy,'Energy or work (kJ)');
        legend(axEnergy,{'mechanical energy change','non-gravity work'}, ...
            'Location','best');
        title(axEnergy,'Work closes the energy ledger');

        cla(axBroken);
        plot(axBroken,out.time_s,out.energyBalanceResidual_J/1e3, ...
            'LineWidth',1.7); hold(axBroken,'on');
        plot(axBroken,out.time_s,out.brokenEnergyBalanceResidual_J/1e3, ...
            '--','LineWidth',1.7);
        grid(axBroken,'on');
        xlabel(axBroken,'Time (s)');
        ylabel(axBroken,'Energy balance residual (kJ)');
        legend(axBroken,{'correct h=-Down','broken h=+Down'}, ...
            'Location','best');
        title(axBroken,'Down-as-height sign failure');

        summary.Text=sprintf([ ...
            'Move one lever and reset: f_x %.2f m/s^2 changes work and the arc; heading %.1f deg actively yaws the path in fixed NED.\n' ...
            'initial v_NED = [%.2f, %.2f, %.2f] m/s | apex gain %.2f m | final work %.3f MJ\n' ...
            'max correct balance %.3g J | body/NED power mismatch %.3g W | broken peak drift %.3f MJ'], ...
            out.forwardSpecificForce_mps2,out.headingAngle_deg, ...
            out.initialVelocityNED_mps(1),out.initialVelocityNED_mps(2), ...
            out.initialVelocityNED_mps(3),out.apexAltitudeGain_m, ...
            out.finalWorkInput_J/1e6,out.maxEnergyBalanceResidual_J, ...
            out.maxPowerFrameDifference_W, ...
            out.peakBrokenEnergyDrift_J/1e6);
    end
end
