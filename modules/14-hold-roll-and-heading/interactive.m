function interactive
%INTERACTIVE Explore P14 heading-to-bank and inner roll-loop levers.
% Move one control, inspect one transition, then reset before moving the next.
clear model;
modelFcn=@model;
uiName='P14 Roll and Heading Hold Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[70 45 1280 900]);
layout=uigridlayout(fig,[6 2]);
layout.RowHeight={72,72,58,36,'1x',120};
layout.ColumnWidth={'1x','1x'};

gainPanel=uipanel(layout,'Title','Lever 1 - outer heading-to-bank gain');
gainPanel.Layout.Row=1;
gainPanel.Layout.Column=[1 2];
gainGrid=uigridlayout(gainPanel,[2 2]);
gainGrid.RowHeight={22,'1x'};
gainGrid.ColumnWidth={315,'1x'};
gainLabel=uilabel(gainGrid, ...
    'Text','K_psi (rad of bank command per rad of heading error)');
gainLabel.Layout.Row=1;
gainLabel.Layout.Column=1;
gainValue=uilabel(gainGrid,'Text','0.50 rad/rad');
gainValue.Layout.Row=2;
gainValue.Layout.Column=1;
gainControl=uislider(gainGrid,'Limits',[0 1],'Value',0.5, ...
    'MajorTicks',[0 0.25 0.5 0.75 1]);
gainControl.Layout.Row=[1 2];
gainControl.Layout.Column=2;

frequencyPanel=uipanel(layout,'Title','Lever 2 - inner roll-loop speed');
frequencyPanel.Layout.Row=2;
frequencyPanel.Layout.Column=[1 2];
frequencyGrid=uigridlayout(frequencyPanel,[2 2]);
frequencyGrid.RowHeight={22,'1x'};
frequencyGrid.ColumnWidth={315,'1x'};
frequencyLabel=uilabel(frequencyGrid, ...
    'Text','Roll natural frequency, omega_phi (rad/s)');
frequencyLabel.Layout.Row=1;
frequencyLabel.Layout.Column=1;
frequencyValue=uilabel(frequencyGrid,'Text','2.4 rad/s');
frequencyValue.Layout.Row=2;
frequencyValue.Layout.Column=1;
frequencyControl=uislider(frequencyGrid,'Limits',[1.2 3.6],'Value',2.4, ...
    'MajorTicks',[1.2 1.8 2.4 3.0 3.6]);
frequencyControl.Layout.Row=[1 2];
frequencyControl.Layout.Column=2;

modePanel=uipanel(layout,'Title','Failure injection - circular heading error');
modePanel.Layout.Row=3;
modePanel.Layout.Column=[1 2];
modeGrid=uigridlayout(modePanel,[1 3]);
modeGrid.ColumnWidth={250,260,'1x'};
uilabel(modeGrid,'Text','Heading-error calculation:');
modeControl=uiswitch(modeGrid,'slider', ...
    'Items',{'Wrapped shortest path','Broken raw subtraction'}, ...
    'Value','Wrapped shortest path');
modeValue=uilabel(modeGrid,'Text','Correct +20 deg circular error');

resetControl=uibutton(layout,'push', ...
    'Text',['Reset: K_psi 0.50 rad/rad, omega_phi 2.4 rad/s, ' ...
    'wrapped shortest-path error']);
resetControl.Layout.Row=4;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=5;
plotGrid.Layout.Column=[1 2];
axHeading=uiaxes(plotGrid);
axError=uiaxes(plotGrid);
axBank=uiaxes(plotGrid);
axHeadingRate=uiaxes(plotGrid);

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
modeControl.ValueChangedFcn=@(source,event) updatePlots();
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        gainControl.Value=0.5;
        frequencyControl.Value=2.4;
        modeControl.Value='Wrapped shortest path';
        updatePlots();
    end

    function updatePlots(event,changingControl)
        headingGain_bank_per_heading=gainControl.Value;
        rollNaturalFrequency_radps=frequencyControl.Value;
        if nargin==2
            switch changingControl
                case 'gain'
                    headingGain_bank_per_heading=event.Value;
                case 'frequency'
                    rollNaturalFrequency_radps=event.Value;
            end
        end
        if strcmp(modeControl.Value,'Broken raw subtraction')
            headingErrorMode=0;
            modeValue.Text='Broken -340 deg raw error and long-way command';
        else
            headingErrorMode=1;
            modeValue.Text='Correct +20 deg circular error';
        end
        out=modelFcn(headingGain_bank_per_heading, ...
            rollNaturalFrequency_radps,headingErrorMode);
        gainValue.Text=sprintf('%.2f rad/rad', ...
            out.headingGain_bank_per_heading);
        frequencyValue.Text=sprintf('%.1f rad/s', ...
            out.rollNaturalFrequency_radps);

        cla(axHeading);
        plot(axHeading,out.time_s,out.headingCommandContinuous_deg,'k--', ...
            'LineWidth',1.3); hold(axHeading,'on');
        plot(axHeading,out.time_s,out.headingUnwrapped_deg,'LineWidth',1.7);
        grid(axHeading,'on');
        xlabel(axHeading,'Time (s)');
        ylabel(axHeading,'Continuous heading near initial (deg)');
        legend(axHeading,{'nearest command','response'},'Location','best');
        title(axHeading,'Continuous heading has no display branch cut');

        cla(axError);
        plot(axError,out.time_s,out.shortestHeadingError_deg,'LineWidth',1.7);
        hold(axError,'on');
        plot(axError,out.time_s,out.headingErrorUsed_deg,':','LineWidth',1.5);
        grid(axError,'on');
        xlabel(axError,'Time (s)'); ylabel(axError,'Heading error (deg)');
        legend(axError,{'independent shortest error','controller-used error'}, ...
            'Location','best');
        title(axError,'Circular error exposes raw-subtraction failure');

        cla(axBank);
        plot(axBank,out.time_s,out.bankCommand_deg,'k--','LineWidth',1.3);
        hold(axBank,'on');
        plot(axBank,out.time_s,out.bankAngle_deg,'LineWidth',1.7);
        plot(axBank,out.time_s, ...
            out.bankCommandLimit_deg*ones(size(out.time_s)),':');
        plot(axBank,out.time_s, ...
            -out.bankCommandLimit_deg*ones(size(out.time_s)),':');
        grid(axBank,'on');
        xlabel(axBank,'Time (s)'); ylabel(axBank,'Bank angle (deg)');
        legend(axBank,{'command','response','+ limit','- limit'}, ...
            'Location','best');
        title(axBank,'Outer command and inner roll response');

        cla(axHeadingRate);
        plot(axHeadingRate,out.time_s,out.headingRate_degps,'LineWidth',1.7);
        grid(axHeadingRate,'on');
        xlabel(axHeadingRate,'Time (s)');
        ylabel(axHeadingRate,'Heading rate (deg/s)');
        title(axHeadingRate,'Coordinated turn rate g tan(phi)/V');

        if out.reachedNinetyPercent
            captureText=sprintf('%.2f s',out.timeToNinetyPercent_s);
        else
            captureText='not reached';
        end
        summary.Text=sprintf([ ...
            'Move one lever, then reset: K_psi %.2f rad/rad changes bank authority; omega_phi %.1f rad/s changes bank tracking/acceleration; mode %d selects wrapped/raw error.\n' ...
            'final shortest error %.3f deg | 90%% time %s | signed/absolute heading travel %.1f/%.1f deg | wrong-way travel %.1f deg\n' ...
            'bank RMS %.3f deg | peak bank %.3f deg | peak bank rate %.3f deg/s | peak bank acceleration %.3f deg/s^2 | saturation %.1f%%'], ...
            out.headingGain_bank_per_heading,out.rollNaturalFrequency_radps, ...
            out.headingErrorMode,out.finalShortestHeadingError_deg, ...
            captureText,out.signedHeadingTravel_deg, ...
            out.absoluteHeadingTravel_deg,out.wrongWayTravel_deg, ...
            out.bankTrackingRMS_deg,out.peakBankAngle_deg, ...
            out.peakBankRate_degps,out.peakBankAcceleration_degps2, ...
            100*out.bankCommandSaturationFraction);
    end
end
