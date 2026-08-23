function interactive
%INTERACTIVE Explore P13 cascaded altitude and pitch-loop levers.
% Move one control, inspect one transition, then reset before moving the next.
clear model;
modelFcn=@model;
uiName='P13 Pitch and Altitude Hold Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[80 50 1260 880]);
layout=uigridlayout(fig,[6 2]);
layout.RowHeight={72,72,58,36,'1x',112};
layout.ColumnWidth={'1x','1x'};

gainPanel=uipanel(layout,'Title','Lever 1 - outer altitude-to-pitch gain');
gainPanel.Layout.Row=1;
gainPanel.Layout.Column=[1 2];
gainGrid=uigridlayout(gainPanel,[2 2]);
gainGrid.RowHeight={22,'1x'};
gainGrid.ColumnWidth={300,'1x'};
gainLabel=uilabel(gainGrid,'Text','K_h (rad of pitch command per m of error)');
gainLabel.Layout.Row=1;
gainLabel.Layout.Column=1;
gainValue=uilabel(gainGrid,'Text','0.0040 rad/m');
gainValue.Layout.Row=2;
gainValue.Layout.Column=1;
gainControl=uislider(gainGrid,'Limits',[0 0.008],'Value',0.004, ...
    'MajorTicks',[0 0.002 0.004 0.006 0.008]);
gainControl.Layout.Row=[1 2];
gainControl.Layout.Column=2;

frequencyPanel=uipanel(layout,'Title','Lever 2 - inner pitch-loop speed');
frequencyPanel.Layout.Row=2;
frequencyPanel.Layout.Column=[1 2];
frequencyGrid=uigridlayout(frequencyPanel,[2 2]);
frequencyGrid.RowHeight={22,'1x'};
frequencyGrid.ColumnWidth={300,'1x'};
frequencyLabel=uilabel(frequencyGrid, ...
    'Text','Pitch natural frequency, omega_n (rad/s)');
frequencyLabel.Layout.Row=1;
frequencyLabel.Layout.Column=1;
frequencyValue=uilabel(frequencyGrid,'Text','2.4 rad/s');
frequencyValue.Layout.Row=2;
frequencyValue.Layout.Column=1;
frequencyControl=uislider(frequencyGrid,'Limits',[1.2 3.6],'Value',2.4, ...
    'MajorTicks',[1.2 1.8 2.4 3.0 3.6]);
frequencyControl.Layout.Row=[1 2];
frequencyControl.Layout.Column=2;

signPanel=uipanel(layout,'Title','Failure injection - outer feedback sign');
signPanel.Layout.Row=3;
signPanel.Layout.Column=[1 2];
signGrid=uigridlayout(signPanel,[1 3]);
signGrid.ColumnWidth={250,220,'1x'};
uilabel(signGrid,'Text','Correct h=-Down or broken Down error:');
signControl=uiswitch(signGrid,'slider', ...
    'Items',{'Correct +1','Broken -1'},'Value','Correct +1');
signValue=uilabel(signGrid,'Text','Correct negative feedback');

resetControl=uibutton(layout,'push', ...
    'Text',['Reset: K_h 0.004 rad/m, omega_n 2.4 rad/s, ' ...
    'correct feedback sign']);
resetControl.Layout.Row=4;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=5;
plotGrid.Layout.Column=[1 2];
axAltitude=uiaxes(plotGrid);
axCascade=uiaxes(plotGrid);
axControl=uiaxes(plotGrid);
axError=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=6;
summary.Layout.Column=[1 2];

gainControl.ValueChangingFcn=@(source,event) updatePlots(event,'gain');
gainControl.ValueChangedFcn=@(source,event) updatePlots(event,'gain');
frequencyControl.ValueChangingFcn= ...
    @(source,event) updatePlots(event,'frequency');
frequencyControl.ValueChangedFcn= ...
    @(source,event) updatePlots(event,'frequency');
signControl.ValueChangedFcn=@(source,event) updatePlots();
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        gainControl.Value=0.004;
        frequencyControl.Value=2.4;
        signControl.Value='Correct +1';
        updatePlots();
    end

    function updatePlots(event,changingControl)
        altitudeGain_rad_per_m=gainControl.Value;
        pitchNaturalFrequency_radps=frequencyControl.Value;
        if nargin==2
            switch changingControl
                case 'gain'
                    altitudeGain_rad_per_m=event.Value;
                case 'frequency'
                    pitchNaturalFrequency_radps=event.Value;
            end
        end
        if strcmp(signControl.Value,'Broken -1')
            altitudeFeedbackSign=-1;
            signValue.Text='Broken positive feedback';
        else
            altitudeFeedbackSign=1;
            signValue.Text='Correct negative feedback';
        end
        out=modelFcn(altitudeGain_rad_per_m, ...
            pitchNaturalFrequency_radps,altitudeFeedbackSign);
        gainValue.Text=sprintf('%.4f rad/m',out.altitudeGain_rad_per_m);
        frequencyValue.Text=sprintf('%.1f rad/s', ...
            out.pitchNaturalFrequency_radps);

        cla(axAltitude);
        plot(axAltitude,out.time_s,out.altitudeCommand_m,'k--', ...
            'LineWidth',1.3); hold(axAltitude,'on');
        plot(axAltitude,out.time_s,out.altitude_m,'LineWidth',1.7);
        grid(axAltitude,'on');
        xlabel(axAltitude,'Time (s)');
        ylabel(axAltitude,'Geometric altitude, h=-Down (m)');
        legend(axAltitude,{'command','response'},'Location','best');
        title(axAltitude,'Outer-loop altitude capture');

        cla(axCascade);
        plot(axCascade,out.time_s,out.pitchCommand_deg,'k--', ...
            'LineWidth',1.2); hold(axCascade,'on');
        plot(axCascade,out.time_s,out.pitchAngle_deg,'LineWidth',1.6);
        plot(axCascade,out.time_s,out.flightPathAngle_deg,':', ...
            'LineWidth',1.6);
        grid(axCascade,'on');
        xlabel(axCascade,'Time (s)'); ylabel(axCascade,'Angle (deg)');
        legend(axCascade,{'pitch command','pitch','flight path'}, ...
            'Location','best');
        title(axCascade,'Pitch leads the flight path');

        cla(axControl);
        plot(axControl,out.time_s,out.pitchControlCommand_deg,'LineWidth',1.6);
        hold(axControl,'on');
        plot(axControl,out.time_s, ...
            out.pitchControlCommandLimit_deg*ones(size(out.time_s)),'k--');
        plot(axControl,out.time_s, ...
            -out.pitchControlCommandLimit_deg*ones(size(out.time_s)),'k--');
        grid(axControl,'on');
        xlabel(axControl,'Time (s)');
        ylabel(axControl,'Equivalent pitch-control command (deg)');
        title(axControl,'Inner-loop authority demand');

        cla(axError);
        plot(axError,out.time_s,out.altitudeError_m,'LineWidth',1.7);
        hold(axError,'on');
        plot(axError,out.time_s, ...
            out.settlingTolerance_m*ones(size(out.time_s)),'k--');
        plot(axError,out.time_s, ...
            -out.settlingTolerance_m*ones(size(out.time_s)),'k--');
        grid(axError,'on');
        xlabel(axError,'Time (s)'); ylabel(axError,'Altitude error (m)');
        title(axError,'Error should contract with the correct sign');

        if out.reachedNinetyPercent
            captureText=sprintf('%.2f s',out.timeToNinetyPercent_s);
        else
            captureText='not reached';
        end
        summary.Text=sprintf([ ...
            'Move one lever, then reset: K_h %.4f rad/m changes capture and overshoot; omega_n %.1f rad/s changes pitch tracking and control demand; sign %+d selects correct/broken feedback.\n' ...
            'final altitude error %.3f m | overshoot %.3f m | 90%% time %s | pitch RMS %.3f deg\n' ...
            'peak pitch %.3f deg | peak path angle %.3f deg | peak pitch-control command %.3f deg | pitch-command saturation %.1f%%'], ...
            out.altitudeGain_rad_per_m,out.pitchNaturalFrequency_radps, ...
            out.altitudeFeedbackSign,out.finalAltitudeError_m, ...
            out.peakAltitudeOvershoot_m,captureText, ...
            out.pitchTrackingRMS_deg,out.peakPitchAngle_deg, ...
            out.peakFlightPathAngle_deg,out.peakPitchControlCommand_deg, ...
            100*out.pitchCommandSaturationFraction);
    end
end
