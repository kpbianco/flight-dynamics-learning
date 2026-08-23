function interactive
%INTERACTIVE Explore P15 speed-gain and throttle-response levers.
% Move one control, inspect one transition, then reset before moving the next.
clear model;
modelFcn=@model;
uiName='P15 Speed and Throttle Control Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[70 45 1280 900]);
layout=uigridlayout(fig,[6 2]);
layout.RowHeight={72,72,58,36,'1x',125};
layout.ColumnWidth={'1x','1x'};

gainPanel=uipanel(layout,'Title','Lever 1 - speed-feedback gain');
gainPanel.Layout.Row=1;
gainPanel.Layout.Column=[1 2];
gainGrid=uigridlayout(gainPanel,[2 2]);
gainGrid.RowHeight={22,'1x'};
gainGrid.ColumnWidth={315,'1x'};
gainLabel=uilabel(gainGrid, ...
    'Text','K_V (desired acceleration per speed error, 1/s)');
gainLabel.Layout.Row=1;
gainLabel.Layout.Column=1;
gainValue=uilabel(gainGrid,'Text','0.150 1/s');
gainValue.Layout.Row=2;
gainValue.Layout.Column=1;
gainControl=uislider(gainGrid,'Limits',[0 0.3],'Value',0.15, ...
    'MajorTicks',[0 0.075 0.15 0.225 0.3]);
gainControl.Layout.Row=[1 2];
gainControl.Layout.Column=2;

timePanel=uipanel(layout,'Title','Lever 2 - delivered-throttle response');
timePanel.Layout.Row=2;
timePanel.Layout.Column=[1 2];
timeGrid=uigridlayout(timePanel,[2 2]);
timeGrid.RowHeight={22,'1x'};
timeGrid.ColumnWidth={315,'1x'};
timeLabel=uilabel(timeGrid, ...
    'Text','Throttle time constant, tau_T (s)');
timeLabel.Layout.Row=1;
timeLabel.Layout.Column=1;
timeValue=uilabel(timeGrid,'Text','0.8 s');
timeValue.Layout.Row=2;
timeValue.Layout.Column=1;
timeControl=uislider(timeGrid,'Limits',[0.2 1.4],'Value',0.8, ...
    'MajorTicks',[0.2 0.5 0.8 1.1 1.4]);
timeControl.Layout.Row=[1 2];
timeControl.Layout.Column=2;

modePanel=uipanel(layout,'Title','Failure injection - speed-feedback sign');
modePanel.Layout.Row=3;
modePanel.Layout.Column=[1 2];
modeGrid=uigridlayout(modePanel,[1 3]);
modeGrid.ColumnWidth={230,290,'1x'};
uilabel(modeGrid,'Text','Feedback calculation:');
modeControl=uiswitch(modeGrid,'slider', ...
    'Items',{'Correct negative feedback','Broken reversed feedback'}, ...
    'Value','Correct negative feedback');
modeValue=uilabel(modeGrid, ...
    'Text','Correct: positive error adds thrust');

resetControl=uibutton(layout,'push', ...
    'Text',['Reset: K_V 0.150 1/s, tau_T 0.8 s, ' ...
    'correct speed feedback']);
resetControl.Layout.Row=4;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=5;
plotGrid.Layout.Column=[1 2];
axSpeed=uiaxes(plotGrid);
axError=uiaxes(plotGrid);
axThrottle=uiaxes(plotGrid);
axForce=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=6;
summary.Layout.Column=[1 2];

gainControl.ValueChangingFcn=@(source,event) updatePlots(event,'gain');
gainControl.ValueChangedFcn=@(source,event) updatePlots(event,'gain');
timeControl.ValueChangingFcn=@(source,event) updatePlots(event,'time');
timeControl.ValueChangedFcn=@(source,event) updatePlots(event,'time');
modeControl.ValueChangedFcn=@(source,event) updatePlots();
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        gainControl.Value=0.15;
        timeControl.Value=0.8;
        modeControl.Value='Correct negative feedback';
        updatePlots();
    end

    function updatePlots(event,changingControl)
        speedGain_per_s=gainControl.Value;
        throttleTimeConstant_s=timeControl.Value;
        if nargin==2
            switch changingControl
                case 'gain'
                    speedGain_per_s=event.Value;
                case 'time'
                    throttleTimeConstant_s=event.Value;
            end
        end
        if strcmp(modeControl.Value,'Broken reversed feedback')
            speedFeedbackSign=-1;
            modeValue.Text='Broken: positive error removes thrust';
        else
            speedFeedbackSign=1;
            modeValue.Text='Correct: positive error adds thrust';
        end
        out=modelFcn(speedGain_per_s,throttleTimeConstant_s, ...
            speedFeedbackSign);
        gainValue.Text=sprintf('%.3f 1/s',out.speedGain_per_s);
        timeValue.Text=sprintf('%.1f s',out.throttleTimeConstant_s);

        cla(axSpeed);
        plot(axSpeed,out.time_s,out.speedCommand_mps,'k--', ...
            'LineWidth',1.3); hold(axSpeed,'on');
        plot(axSpeed,out.time_s,out.trueAirspeed_mps,'LineWidth',1.7);
        plot(axSpeed,out.time_s, ...
            out.stallSpeed_mps*ones(size(out.time_s)),'r-.','LineWidth',1.1);
        grid(axSpeed,'on');
        xlabel(axSpeed,'Time (s)'); ylabel(axSpeed,'True airspeed (m/s)');
        legend(axSpeed,{'command','response','stall boundary'}, ...
            'Location','best');
        title(axSpeed,'Speed response and declared envelope');

        cla(axError);
        plot(axError,out.time_s,out.speedError_mps,'LineWidth',1.7);
        hold(axError,'on');
        plot(axError,out.time_s,out.speedErrorUsed_mps,':','LineWidth',1.5);
        grid(axError,'on');
        xlabel(axError,'Time (s)'); ylabel(axError,'Speed error (m/s)');
        legend(axError,{'proper command-minus-speed','controller-used'}, ...
            'Location','best');
        title(axError,'Feedback sign decides whether error contracts');

        cla(axThrottle);
        plot(axThrottle,out.time_s,100*out.throttleCommand,'k--', ...
            'LineWidth',1.3); hold(axThrottle,'on');
        plot(axThrottle,out.time_s,100*out.throttleActual,'LineWidth',1.7);
        grid(axThrottle,'on');
        xlabel(axThrottle,'Time (s)'); ylabel(axThrottle,'Throttle (%)');
        legend(axThrottle,{'command','delivered'},'Location','best');
        title(axThrottle,'Requested and delivered throttle');

        cla(axForce);
        plot(axForce,out.time_s,out.thrustActual_N,'LineWidth',1.7);
        hold(axForce,'on');
        plot(axForce,out.time_s,out.drag_N,'LineWidth',1.7);
        grid(axForce,'on');
        xlabel(axForce,'Time (s)'); ylabel(axForce,'Force (N)');
        legend(axForce,{'delivered thrust','drag'},'Location','best');
        title(axForce,'Net force changes airspeed');

        if out.reachedNinetyPercent
            captureText=sprintf('%.2f s',out.timeToNinetyPercent_s);
        else
            captureText='not reached';
        end
        summary.Text=sprintf([ ...
            'Move one lever, then reset: K_V %.3f 1/s changes corrective-force authority; tau_T %.1f s changes delivery lag/rate; sign %+d selects correct/reversed feedback.\n' ...
            'speed at 5 s %.3f m/s | final error %.3f m/s | 90%% time %s | speed RMS %.3f m/s | minimum stall margin %.3f m/s\n' ...
            'peak commanded/delivered throttle %.1f/%.1f%% | peak rate %.3f 1/s | peak |acceleration| %.3f m/s^2 | saturation %.1f%%'], ...
            out.speedGain_per_s,out.throttleTimeConstant_s, ...
            out.speedFeedbackSign,out.speedAtFiveSeconds_mps, ...
            out.finalSpeedError_mps,captureText,out.speedTrackingRMS_mps, ...
            out.minimumStallMargin_mps,100*out.peakThrottleCommand, ...
            100*out.peakThrottleActual,out.peakAbsoluteThrottleRate_per_s, ...
            out.peakAbsoluteAcceleration_mps2, ...
            100*out.thrustCommandSaturationFraction);
    end
end
