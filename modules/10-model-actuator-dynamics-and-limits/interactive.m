function interactive
%INTERACTIVE Explore P10 actuator lag, rate authority, and hard-stop behavior.
% Move one control, inspect one transition, then reset before moving the next.
clear model;
modelFcn=@model;
uiName='P10 Actuator Dynamics Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[100 100 1180 780]);
layout=uigridlayout(fig,[5 2]);
layout.RowHeight={72,72,36,'1x',82};
layout.ColumnWidth={'1x','1x'};

timePanel=uipanel(layout,'Title','Lever 1 - actuator time constant');
timePanel.Layout.Row=1;
timePanel.Layout.Column=[1 2];
timeGrid=uigridlayout(timePanel,[2 2]);
timeGrid.RowHeight={22,'1x'};
timeGrid.ColumnWidth={210,'1x'};
timeLabel=uilabel(timeGrid,'Text','Time constant tau (s)');
timeLabel.Layout.Row=1;
timeLabel.Layout.Column=1;
timeValue=uilabel(timeGrid,'Text','0.18 s');
timeValue.Layout.Row=2;
timeValue.Layout.Column=1;
timeControl=uislider(timeGrid,'Limits',[0.05 0.50],'Value',0.18, ...
    'MajorTicks',[0.05 0.10 0.18 0.30 0.40 0.50]);
timeControl.Layout.Row=[1 2];
timeControl.Layout.Column=2;

ratePanel=uipanel(layout,'Title','Lever 2 - symmetric rate authority');
ratePanel.Layout.Row=2;
ratePanel.Layout.Column=[1 2];
rateGrid=uigridlayout(ratePanel,[2 2]);
rateGrid.RowHeight={22,'1x'};
rateGrid.ColumnWidth={210,'1x'};
rateLabel=uilabel(rateGrid,'Text','Rate limit (deg/s)');
rateLabel.Layout.Row=1;
rateLabel.Layout.Column=1;
rateValue=uilabel(rateGrid,'Text','45 deg/s');
rateValue.Layout.Row=2;
rateValue.Layout.Column=1;
rateControl=uislider(rateGrid,'Limits',[20 120],'Value',45, ...
    'MajorTicks',[20 30 45 60 80 100 120]);
rateControl.Layout.Row=[1 2];
rateControl.Layout.Column=2;

resetControl=uibutton(layout,'push', ...
    'Text','Reset both levers to baseline (tau 0.18 s, rate 45 deg/s)');
resetControl.Layout.Row=3;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=4;
plotGrid.Layout.Column=[1 2];
axDeflection=uiaxes(plotGrid);
axRate=uiaxes(plotGrid);
axMoment=uiaxes(plotGrid);
axBroken=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=5;
summary.Layout.Column=[1 2];

timeControl.ValueChangingFcn=@(source,event) updatePlots(event,'time');
timeControl.ValueChangedFcn=@(source,event) updatePlots(event,'time');
rateControl.ValueChangingFcn=@(source,event) updatePlots(event,'rate');
rateControl.ValueChangedFcn=@(source,event) updatePlots(event,'rate');
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        timeControl.Value=0.18;
        rateControl.Value=45;
        updatePlots();
    end

    function updatePlots(event,changingControl)
        timeConstant_s=timeControl.Value;
        rateLimit_deg_s=rateControl.Value;
        if nargin==2
            switch changingControl
                case 'time'
                    timeConstant_s=event.Value;
                case 'rate'
                    rateLimit_deg_s=event.Value;
            end
        end
        out=modelFcn(timeConstant_s,rateLimit_deg_s);
        timeValue.Text=sprintf('%.2f s',out.timeConstant_s);
        rateValue.Text=sprintf('%.0f deg/s',out.rateLimit_deg_s);

        cla(axDeflection);
        plot(axDeflection,out.time_s,out.command_deg,'k:', ...
            'LineWidth',1.3); hold(axDeflection,'on');
        plot(axDeflection,out.time_s,out.limitedCommand_deg,'--', ...
            'LineWidth',1.4);
        plot(axDeflection,out.time_s,out.deflection_deg,'LineWidth',1.7);
        plot(axDeflection,out.time_s,out.positionLimit_deg* ...
            ones(size(out.time_s)),'r-.');
        plot(axDeflection,out.time_s,-out.positionLimit_deg* ...
            ones(size(out.time_s)),'r-.');
        grid(axDeflection,'on');
        xlabel(axDeflection,'Time (s)');
        ylabel(axDeflection,'Deflection (deg)');
        legend(axDeflection,{'requested','within stop','delivered', ...
            '+ stop','- stop'},'Location','best');
        title(axDeflection,'Command to delivered surface');

        cla(axRate);
        plot(axRate,out.time_s,out.lagRateDemand_deg_s,'--', ...
            'LineWidth',1.3); hold(axRate,'on');
        plot(axRate,out.time_s,out.actualRate_deg_s,'LineWidth',1.7);
        plot(axRate,out.time_s,out.rateLimit_deg_s* ...
            ones(size(out.time_s)),'r-.');
        plot(axRate,out.time_s,-out.rateLimit_deg_s* ...
            ones(size(out.time_s)),'r-.');
        grid(axRate,'on');
        xlabel(axRate,'Time (s)');
        ylabel(axRate,'Actuator rate (deg/s)');
        legend(axRate,{'lag demand','delivered','+ limit','- limit'}, ...
            'Location','best');
        title(axRate,'Lag demand versus rate authority');

        cla(axMoment);
        plot(axMoment,out.time_s,out.requestedPitchMoment_Nm,'k:', ...
            'LineWidth',1.3); hold(axMoment,'on');
        plot(axMoment,out.time_s,out.feasiblePitchMoment_Nm,'--', ...
            'LineWidth',1.4);
        plot(axMoment,out.time_s,out.deliveredPitchMoment_Nm, ...
            'LineWidth',1.7);
        grid(axMoment,'on');
        xlabel(axMoment,'Time (s)');
        ylabel(axMoment,'Pitch moment M_y (N*m)');
        legend(axMoment,{'requested','within stop','delivered'}, ...
            'Location','best');
        title(axMoment,'Conceptual body-y moment ledger');

        cla(axBroken);
        plot(axBroken,out.time_s,out.deflection_deg,'LineWidth',1.7); hold(axBroken,'on');
        plot(axBroken,out.time_s,out.brokenDeflection_deg,'--','LineWidth',1.7);
        plot(axBroken,out.time_s,out.positionLimit_deg* ...
            ones(size(out.time_s)),'r-.');
        plot(axBroken,out.time_s,-out.positionLimit_deg* ...
            ones(size(out.time_s)),'r-.');
        grid(axBroken,'on');
        xlabel(axBroken,'Time (s)');
        ylabel(axBroken,'Deflection (deg)');
        legend(axBroken,{'complete','position envelope omitted', ...
            '+ stop','- stop'},'Location','best');
        title(axBroken,'Broken omitted-position-envelope comparison');

        summary.Text=sprintf([ ...
            'Move one lever and reset: tau %.2f s changes lag only; rate %.0f deg/s changes motion authority only; hard stop remains +/-%.0f deg.\n' ...
            '90%% of feasible +%.0f deg target in %.2f s | reversal zero crossing %.2f s | feasible RMS error %.2f deg | rate-limited %.2f s\n' ...
            'peak delivered moment %.0f N*m | complete excess %.2g deg | broken excess %.2f deg | invented peak moment %.0f N*m'], ...
            out.timeConstant_s,out.rateLimit_deg_s,out.positionLimit_deg, ...
            out.positionLimit_deg, ...
            out.positiveNinetyResponseTime_s, ...
            out.reversalZeroCrossingDelay_s, ...
            out.rmsFeasibleTrackingError_deg,out.rateLimitedDuration_s, ...
            out.peakDeliveredPitchMoment_Nm, ...
            max(out.positionLimitExcess_deg), ...
            out.brokenMaximumPositionExcess_deg, ...
            out.brokenPeakMomentExcess_Nm);
    end
end
