function interactive
%INTERACTIVE Explore P17 INS/GPS prediction, correction, and gating.
% Move one sensor-error lever, inspect one transition, then reset.
clear model;
modelFcn=@model;
uiName='P17 INS GPS Fusion Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[70 35 1320 930]);
layout=uigridlayout(fig,[6 2]);
layout.RowHeight={72,72,64,36,'1x',155};
layout.ColumnWidth={'1x','1x'};

biasPanel=uipanel(layout,'Title','Lever 1 - INS acceleration bias');
biasPanel.Layout.Row=1;
biasPanel.Layout.Column=[1 2];
biasGrid=uigridlayout(biasPanel,[2 2]);
biasGrid.RowHeight={22,'1x'};
biasGrid.ColumnWidth={320,'1x'};
uilabel(biasGrid,'Text','Residual North acceleration bias (m/s^2)');
biasValue=uilabel(biasGrid,'Text','0.040 m/s^2');
biasValue.Layout.Row=2;
biasValue.Layout.Column=1;
biasControl=uislider(biasGrid,'Limits',[-0.08 0.08], ...
    'Value',0.04,'MajorTicks',[-0.08 -0.04 0 0.04 0.08]);
biasControl.Layout.Row=[1 2];
biasControl.Layout.Column=2;

gpsPanel=uipanel(layout,'Title','Lever 2 - GPS position-error RMS');
gpsPanel.Layout.Row=2;
gpsPanel.Layout.Column=[1 2];
gpsGrid=uigridlayout(gpsPanel,[2 2]);
gpsGrid.RowHeight={22,'1x'};
gpsGrid.ColumnWidth={320,'1x'};
uilabel(gpsGrid,'Text','Deterministic nominal GPS error RMS (m)');
gpsValue=uilabel(gpsGrid,'Text','1.00 m');
gpsValue.Layout.Row=2;
gpsValue.Layout.Column=1;
gpsControl=uislider(gpsGrid,'Limits',[0 4], ...
    'Value',1,'MajorTicks',[0 0.5 1 2 3 4]);
gpsControl.Layout.Row=[1 2];
gpsControl.Layout.Column=2;

modePanel=uipanel(layout,'Title','Fusion path and deliberately broken gate');
modePanel.Layout.Row=3;
modePanel.Layout.Column=[1 2];
modeGrid=uigridlayout(modePanel,[1 3]);
modeGrid.ColumnWidth={160,330,'1x'};
uilabel(modeGrid,'Text','Estimator mode:');
modeControl=uidropdown(modeGrid,'Items',{ ...
    'Gated INS/GPS fusion', ...
    'INS-only limiting case', ...
    'BROKEN accept-all fusion'}, ...
    'Value','Gated INS/GPS fusion');
modeValue=uilabel(modeGrid, ...
    'Text','Correct: reject |GPS innovation| above 25 m');

resetControl=uibutton(layout,'push', ...
    'Text',['Reset: bias 0.040 m/s^2, GPS RMS 1.00 m, ' ...
    'gated INS/GPS fusion']);
resetControl.Layout.Row=4;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=5;
plotGrid.Layout.Column=[1 2];
axPosition=uiaxes(plotGrid);
axError=uiaxes(plotGrid);
axInnovation=uiaxes(plotGrid);
axVelocity=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=6;
summary.Layout.Column=[1 2];

biasControl.ValueChangingFcn=@(source,event) updatePlots(event,'bias');
biasControl.ValueChangedFcn=@(source,event) updatePlots(event,'bias');
gpsControl.ValueChangingFcn=@(source,event) updatePlots(event,'gps');
gpsControl.ValueChangedFcn=@(source,event) updatePlots(event,'gps');
modeControl.ValueChangedFcn=@(source,event) updatePlots();
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        biasControl.Value=0.04;
        gpsControl.Value=1;
        modeControl.Value='Gated INS/GPS fusion';
        updatePlots();
    end

    function updatePlots(event,changingControl)
        insAccelerationBias_mps2=biasControl.Value;
        gpsPositionErrorRms_m=gpsControl.Value;
        if nargin==2
            switch changingControl
                case 'bias'
                    insAccelerationBias_mps2=event.Value;
                case 'gps'
                    gpsPositionErrorRms_m=event.Value;
            end
        end

        switch modeControl.Value
            case 'Gated INS/GPS fusion'
                fusionMode=1;
                modeValue.Text= ...
                    'Correct: reject |GPS innovation| above 25 m';
            case 'INS-only limiting case'
                fusionMode=0;
                modeValue.Text= ...
                    'Limit: GPS fixes are present but corrections are ignored';
            otherwise
                fusionMode=-1;
                modeValue.Text= ...
                    'Broken: accept every GPS fix, including the +80 m outlier';
        end

        out=modelFcn(insAccelerationBias_mps2, ...
            gpsPositionErrorRms_m,fusionMode);
        gated=modelFcn(insAccelerationBias_mps2, ...
            gpsPositionErrorRms_m,1);
        biasValue.Text=sprintf('%.3f m/s^2', ...
            out.insAccelerationBias_mps2);
        gpsValue.Text=sprintf('%.2f m',out.gpsPositionErrorRms_m);

        cla(axPosition);
        plot(axPosition,out.time_s,out.northPositionTruth_m,'k-', ...
            'LineWidth',1.7);
        hold(axPosition,'on');
        plot(axPosition,out.time_s,out.northPositionINSOnly_m,'--', ...
            'LineWidth',1.3);
        plot(axPosition,out.time_s,out.northPositionFused_m, ...
            'LineWidth',1.5);
        plot(axPosition,out.time_s(out.gpsUpdateMask), ...
            out.gpsPositionMeasurement_m(out.gpsUpdateMask),'o', ...
            'MarkerSize',2.5);
        grid(axPosition,'on');
        xlabel(axPosition,'Time (s)');
        ylabel(axPosition,'North position (m)');
        legend(axPosition,{'truth','INS only','selected estimator','GPS fixes'}, ...
            'Location','best');
        title(axPosition,'Truth, dead reckoning, fixes, and estimate');

        cla(axError);
        plot(axError,out.time_s,out.northPositionINSOnlyError_m,'--', ...
            'LineWidth',1.3);
        hold(axError,'on');
        plot(axError,out.time_s,out.northPositionFusedError_m, ...
            'LineWidth',1.6);
        if fusionMode~=1
            plot(axError,gated.time_s,gated.northPositionFusedError_m,':', ...
                'LineWidth',1.4);
            errorLegend={'INS only','selected estimator','gated comparison'};
        else
            errorLegend={'INS only','gated fusion'};
        end
        grid(axError,'on');
        xlabel(axError,'Time (s)');
        ylabel(axError,'North position error (m)');
        legend(axError,errorLegend,'Location','best');
        title(axError,'Bias drift versus corrected error');

        cla(axInnovation);
        stem(axInnovation,out.time_s(out.gpsUpdateMask), ...
            out.gpsInnovation_m(out.gpsUpdateMask),'filled', ...
            'MarkerSize',2.5);
        hold(axInnovation,'on');
        yline(axInnovation,out.innovationGate_m,'r--');
        yline(axInnovation,-out.innovationGate_m,'r--');
        grid(axInnovation,'on');
        xlabel(axInnovation,'Time (s)');
        ylabel(axInnovation,'GPS innovation (m)');
        title(axInnovation,'Residual and inclusive innovation gate');

        cla(axVelocity);
        plot(axVelocity,out.time_s,out.northVelocityINSOnlyError_mps,'--', ...
            'LineWidth',1.3);
        hold(axVelocity,'on');
        plot(axVelocity,out.time_s,out.northVelocityFusedError_mps, ...
            'LineWidth',1.6);
        stem(axVelocity,out.time_s(out.gpsAccepted), ...
            out.gpsVelocityCorrection_mps(out.gpsAccepted),'.');
        grid(axVelocity,'on');
        xlabel(axVelocity,'Time (s)');
        ylabel(axVelocity,'Velocity error/correction (m/s)');
        legend(axVelocity,{'INS-only error','fused error', ...
            'accepted correction'},'Location','best');
        title(axVelocity,'Position residual also corrects velocity');

        summary.Text=sprintf([ ...
            'Move one lever, then reset: bias %.3f m/s^2 | selected GPS RMS %.2f m | measured nominal RMS %.2f m\n' ...
            '%s | alpha %.2f | beta %.2f | gate +/-%.1f m | fixes used/rejected/ignored %d/%d/%d\n' ...
            'INS-only final error %.2f m and %.2f m/s | fused RMS %.2f m and %.2f m/s | fused peak %.2f m\n' ...
            'outlier innovation %.2f m | position correction %.2f m | velocity correction %.2f m/s'], ...
            out.insAccelerationBias_mps2,out.gpsPositionErrorRms_m, ...
            out.gpsPositionErrorRmsMeasured_m,out.fusionModeName, ...
            out.gpsPositionGain,out.gpsVelocityGain, ...
            out.innovationGate_m,out.gpsAcceptedCount, ...
            out.gpsRejectedCount,out.gpsIgnoredCount, ...
            out.insOnlyFinalPositionError_m, ...
            out.insOnlyFinalVelocityError_mps,out.fusedPositionRMS_m, ...
            out.fusedVelocityRMS_mps, ...
            out.fusedPeakAbsolutePositionError_m, ...
            out.outlierInnovation_m,out.outlierPositionCorrection_m, ...
            out.outlierVelocityCorrection_mps);
    end
end
