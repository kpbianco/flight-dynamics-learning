function interactive
%INTERACTIVE Explore P16 flight-condition gain scheduling controls.
% Move one condition lever, inspect one transition, then reset before the next.
clear model;
modelFcn=@model;
referenceDensity=0.736115547399152;
uiName='P16 Flight-Condition Gain Schedule Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[65 40 1300 920]);
layout=uigridlayout(fig,[6 2]);
layout.RowHeight={72,72,64,36,'1x',145};
layout.ColumnWidth={'1x','1x'};

speedPanel=uipanel(layout,'Title','Lever 1 - true airspeed');
speedPanel.Layout.Row=1;
speedPanel.Layout.Column=[1 2];
speedGrid=uigridlayout(speedPanel,[2 2]);
speedGrid.RowHeight={22,'1x'};
speedGrid.ColumnWidth={320,'1x'};
speedLabel=uilabel(speedGrid, ...
    'Text','True airspeed V (m/s), one input to qbar');
speedLabel.Layout.Row=1;
speedLabel.Layout.Column=1;
speedValue=uilabel(speedGrid,'Text','60.0 m/s');
speedValue.Layout.Row=2;
speedValue.Layout.Column=1;
speedControl=uislider(speedGrid,'Limits',[45 75],'Value',60, ...
    'MajorTicks',[45 52.5 60 67.5 75]);
speedControl.Layout.Row=[1 2];
speedControl.Layout.Column=2;

densityPanel=uipanel(layout,'Title','Lever 2 - air density');
densityPanel.Layout.Row=2;
densityPanel.Layout.Column=[1 2];
densityGrid=uigridlayout(densityPanel,[2 2]);
densityGrid.RowHeight={22,'1x'};
densityGrid.ColumnWidth={320,'1x'};
densityLabel=uilabel(densityGrid, ...
    'Text','Air density rho (kg/m^3), the other input to qbar');
densityLabel.Layout.Row=1;
densityLabel.Layout.Column=1;
densityValue=uilabel(densityGrid,'Text','0.736116 kg/m^3');
densityValue.Layout.Row=2;
densityValue.Layout.Column=1;
densityControl=uislider(densityGrid, ...
    'Limits',[0.5*referenceDensity 1.5*referenceDensity], ...
    'Value',referenceDensity, ...
    'MajorTicks',referenceDensity*[0.5 0.75 1 1.25 1.5]);
densityControl.Layout.Row=[1 2];
densityControl.Layout.Column=2;

modePanel=uipanel(layout, ...
    'Title','Schedule source and deliberately broken lookup');
modePanel.Layout.Row=3;
modePanel.Layout.Column=[1 2];
modeGrid=uigridlayout(modePanel,[1 3]);
modeGrid.ColumnWidth={215,330,'1x'};
uilabel(modeGrid,'Text','Gain selection:');
modeControl=uidropdown(modeGrid,'Items',{ ...
    'Dynamic-pressure schedule', ...
    'Fixed reference gains', ...
    'BROKEN true-airspeed-only lookup'}, ...
    'Value','Dynamic-pressure schedule');
modeValue=uilabel(modeGrid, ...
    'Text','Correct: qbar uses both true airspeed and density');

resetControl=uibutton(layout,'push', ...
    'Text',['Reset: V 60 m/s, rho 0.736116 kg/m^3, ' ...
    'dynamic-pressure schedule']);
resetControl.Layout.Row=4;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=5;
plotGrid.Layout.Column=[1 2];
axRoll=uiaxes(plotGrid);
axError=uiaxes(plotGrid);
axControl=uiaxes(plotGrid);
axSchedule=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=6;
summary.Layout.Column=[1 2];

speedControl.ValueChangingFcn=@(source,event) updatePlots(event,'speed');
speedControl.ValueChangedFcn=@(source,event) updatePlots(event,'speed');
densityControl.ValueChangingFcn=@(source,event) updatePlots(event,'density');
densityControl.ValueChangedFcn=@(source,event) updatePlots(event,'density');
modeControl.ValueChangedFcn=@(source,event) updatePlots();
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        speedControl.Value=60;
        densityControl.Value=referenceDensity;
        modeControl.Value='Dynamic-pressure schedule';
        updatePlots();
    end

    function updatePlots(event,changingControl)
        trueAirspeed_mps=speedControl.Value;
        airDensity_kgpm3=densityControl.Value;
        if nargin==2
            switch changingControl
                case 'speed'
                    trueAirspeed_mps=event.Value;
                case 'density'
                    airDensity_kgpm3=event.Value;
            end
        end

        switch modeControl.Value
            case 'Dynamic-pressure schedule'
                scheduleMode=1;
                modeValue.Text= ...
                    'Correct: qbar uses both true airspeed and density';
            case 'Fixed reference gains'
                scheduleMode=0;
                modeValue.Text= ...
                    'Comparison: always select the qbar/qbar_ref=1 gains';
            otherwise
                scheduleMode=-1;
                modeValue.Text= ...
                    'Broken: density is omitted from the lookup variable';
        end

        out=modelFcn(trueAirspeed_mps,airDensity_kgpm3,scheduleMode);
        fixed=modelFcn(trueAirspeed_mps,airDensity_kgpm3,0);
        speedValue.Text=sprintf('%.1f m/s',out.trueAirspeed_mps);
        densityValue.Text=sprintf('%.6f kg/m^3',out.airDensity_kgpm3);

        cla(axRoll);
        plot(axRoll,out.time_s,out.rollCommand_deg,'k--','LineWidth',1.3);
        hold(axRoll,'on');
        plot(axRoll,out.time_s,out.rollAngle_deg,'LineWidth',1.7);
        if scheduleMode~=0
            plot(axRoll,fixed.time_s,fixed.rollAngle_deg,':','LineWidth',1.5);
            rollLegend={'command','selected schedule','fixed comparison'};
        else
            rollLegend={'command','fixed reference gains'};
        end
        grid(axRoll,'on');
        xlabel(axRoll,'Time (s)'); ylabel(axRoll,'Roll angle (deg)');
        legend(axRoll,rollLegend,'Location','best');
        title(axRoll,'Flight-condition roll response');

        cla(axError);
        plot(axError,out.time_s,out.rollError_deg,'LineWidth',1.7);
        hold(axError,'on');
        yline(axError,out.settlingTolerance_deg,'k:');
        yline(axError,-out.settlingTolerance_deg,'k:');
        grid(axError,'on');
        xlabel(axError,'Time (s)'); ylabel(axError,'Roll error (deg)');
        title(axError,'Tracking error and 2% band');

        cla(axControl);
        plot(axControl,out.time_s,out.aileronCommand_deg,'LineWidth',1.7);
        hold(axControl,'on');
        if scheduleMode~=0
            plot(axControl,fixed.time_s,fixed.aileronCommand_deg,':', ...
                'LineWidth',1.5);
            controlLegend={'selected schedule','fixed comparison', ...
                'positive limit','negative limit'};
        else
            controlLegend={'fixed reference gains', ...
                'positive limit','negative limit'};
        end
        yline(axControl,out.aileronCommandLimit_deg,'r--');
        yline(axControl,-out.aileronCommandLimit_deg,'r--');
        grid(axControl,'on');
        xlabel(axControl,'Time (s)');
        ylabel(axControl,'Equivalent aileron command (deg)');
        legend(axControl,controlLegend,'Location','best');
        title(axControl,'Gain choice changes control demand');

        cla(axSchedule);
        plot(axSchedule,out.dynamicPressureRatioKnots, ...
            out.rollAngleGainTable,'o-','LineWidth',1.5);
        hold(axSchedule,'on');
        plot(axSchedule,out.dynamicPressureRatioKnots, ...
            out.rollRateGainTable_s,'s-','LineWidth',1.5);
        plot(axSchedule,out.lookupDynamicPressureRatio, ...
            out.rollAngleGain,'ko','MarkerFaceColor','k');
        xline(axSchedule,out.actualDynamicPressureRatio,'b:','LineWidth',1.3);
        grid(axSchedule,'on');
        xlabel(axSchedule,'Dynamic-pressure ratio qbar/qbar_{ref}');
        ylabel(axSchedule,'Gain (rad/rad or s)');
        legend(axSchedule,{'K_phi table','K_p table','selected K_phi', ...
            'actual plant ratio'},'Location','best');
        title(axSchedule,'Ordered knots, lookup, and actual condition');

        if out.lookupClamped
            clampText='yes - outside table, endpoint held';
        else
            clampText='no';
        end
        summary.Text=sprintf([ ...
            'Move one lever, then reset: V %.1f m/s | rho %.6f kg/m^3 | qbar %.2f Pa | actual ratio %.4f\n' ...
            'source: %s | raw lookup %.4f | used lookup %.4f | clamped %s | bracket %d-%d | weight %.4f\n' ...
            'K_phi %.5f rad/rad | K_p %.5f s | effective omega_n %.4f rad/s | zeta %.4f\n' ...
            'table error at used lookup %.2f%% | selected K_phi versus actual-condition ideal %.2f%%\n' ...
            '90%% time %.2f s | settling %.2f s | overshoot %.3f deg | peak aileron %.3f deg | saturation %.1f%%'], ...
            out.trueAirspeed_mps,out.airDensity_kgpm3, ...
            out.actualDynamicPressure_Pa,out.actualDynamicPressureRatio, ...
            out.scheduleSource,out.lookupDynamicPressureRatioRaw, ...
            out.lookupDynamicPressureRatio,clampText,out.lowerKnotIndex, ...
            out.upperKnotIndex,out.interpolationWeight,out.rollAngleGain, ...
            out.rollRateGain_s,out.effectiveNaturalFrequency_radps, ...
            out.effectiveDampingRatio, ...
            100*out.rollAngleGainInterpolationErrorFraction, ...
            100*out.rollAngleGainActualConditionMismatchFraction, ...
            out.timeToNinetyPercent_s,out.settlingTime_s, ...
            out.peakRollOvershoot_deg, ...
            out.peakAbsoluteAileronCommand_deg, ...
            100*out.aileronCommandSaturationFraction);
    end
end
