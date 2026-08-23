function interactive
%INTERACTIVE Explore P11 gyro bias and accelerometer teaching-error magnitude.
% Move one control, inspect one transition, then reset before moving the next.
clear model;
modelFcn=@model;
uiName='P11 Flight Sensor Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[100 80 1200 820]);
layout=uigridlayout(fig,[5 2]);
layout.RowHeight={72,72,36,'1x',92};
layout.ColumnWidth={'1x','1x'};

biasPanel=uipanel(layout,'Title','Lever 1 - constant pitch-gyro bias');
biasPanel.Layout.Row=1;
biasPanel.Layout.Column=[1 2];
biasGrid=uigridlayout(biasPanel,[2 2]);
biasGrid.RowHeight={22,'1x'};
biasGrid.ColumnWidth={240,'1x'};
biasLabel=uilabel(biasGrid,'Text','Gyro bias (deg/s)');
biasLabel.Layout.Row=1;
biasLabel.Layout.Column=1;
biasValue=uilabel(biasGrid,'Text','0.20 deg/s');
biasValue.Layout.Row=2;
biasValue.Layout.Column=1;
biasControl=uislider(biasGrid,'Limits',[-0.5 0.5],'Value',0.20, ...
    'MajorTicks',[-0.5 -0.25 0 0.20 0.5]);
biasControl.Layout.Row=[1 2];
biasControl.Layout.Column=2;

noisePanel=uipanel(layout, ...
    'Title','Lever 2 - accelerometer additive-error vector RMS');
noisePanel.Layout.Row=2;
noisePanel.Layout.Column=[1 2];
noiseGrid=uigridlayout(noisePanel,[2 2]);
noiseGrid.RowHeight={22,'1x'};
noiseGrid.ColumnWidth={240,'1x'};
noiseLabel=uilabel(noiseGrid,'Text','Vector RMS (m/s^2)');
noiseLabel.Layout.Row=1;
noiseLabel.Layout.Column=1;
noiseValue=uilabel(noiseGrid,'Text','0.15 m/s^2');
noiseValue.Layout.Row=2;
noiseValue.Layout.Column=1;
noiseControl=uislider(noiseGrid,'Limits',[0 0.5],'Value',0.15, ...
    'MajorTicks',[0 0.05 0.15 0.30 0.50]);
noiseControl.Layout.Row=[1 2];
noiseControl.Layout.Column=2;

resetControl=uibutton(layout,'push', ...
    'Text',['Reset both levers to baseline (bias 0.20 deg/s, ' ...
    'accelerometer RMS 0.15 m/s^2)']);
resetControl.Layout.Row=3;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=4;
plotGrid.Layout.Column=[1 2];
axGyro=uiaxes(plotGrid);
axAngle=uiaxes(plotGrid);
axAccelerometer=uiaxes(plotGrid);
axBroken=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=5;
summary.Layout.Column=[1 2];

biasControl.ValueChangingFcn=@(source,event) updatePlots(event,'bias');
biasControl.ValueChangedFcn=@(source,event) updatePlots(event,'bias');
noiseControl.ValueChangingFcn=@(source,event) updatePlots(event,'noise');
noiseControl.ValueChangedFcn=@(source,event) updatePlots(event,'noise');
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        biasControl.Value=0.20;
        noiseControl.Value=0.15;
        updatePlots();
    end

    function updatePlots(event,changingControl)
        gyroBias_deg_s=biasControl.Value;
        accelerometerNoiseRms_mps2=noiseControl.Value;
        if nargin==2
            switch changingControl
                case 'bias'
                    gyroBias_deg_s=event.Value;
                case 'noise'
                    accelerometerNoiseRms_mps2=event.Value;
            end
        end
        out=modelFcn(gyroBias_deg_s,accelerometerNoiseRms_mps2);
        biasValue.Text=sprintf('%.2f deg/s',out.gyroBias_deg_s);
        noiseValue.Text=sprintf('%.2f m/s^2', ...
            out.accelerometerNoiseRms_mps2);

        cla(axGyro);
        plot(axGyro,out.time_s,out.pitchRateTruth_deg_s,'k--', ...
            'LineWidth',1.3); hold(axGyro,'on');
        plot(axGyro,out.time_s,out.pitchRateMeasured_deg_s, ...
            'LineWidth',1.7);
        grid(axGyro,'on');
        xlabel(axGyro,'Time (s)');
        ylabel(axGyro,'Pitch rate q (deg/s)');
        legend(axGyro,{'truth','gyro measurement'},'Location','best');
        title(axGyro,'Bias changes gyro only');

        cla(axAngle);
        plot(axAngle,out.time_s,out.pitchAngleError_deg,'LineWidth',1.7); hold(axAngle,'on');
        plot(axAngle,out.time_s,out.expectedPitchAngleError_deg, ...
            'k--','LineWidth',1.2);
        grid(axAngle,'on');
        xlabel(axAngle,'Time (s)');
        ylabel(axAngle,'Pitch angle error (deg)');
        legend(axAngle,{'integrated error','bias*time'},'Location','best');
        title(axAngle,'Constant rate error accumulates');

        cla(axAccelerometer);
        plot(axAccelerometer,out.time_s, ...
            out.idealSpecificForceBody_mps2(1,:),'k--','LineWidth',1.3); hold(axAccelerometer,'on');
        plot(axAccelerometer,out.time_s, ...
            out.accelerometerMeasuredBody_mps2(1,:),'LineWidth',1.5);
        plot(axAccelerometer,out.time_s, ...
            out.idealSpecificForceBody_mps2(3,:),'--','LineWidth',1.3);
        plot(axAccelerometer,out.time_s, ...
            out.accelerometerMeasuredBody_mps2(3,:),'LineWidth',1.5);
        grid(axAccelerometer,'on');
        xlabel(axAccelerometer,'Time (s)');
        ylabel(axAccelerometer,'Body specific force (m/s^2)');
        legend(axAccelerometer,{'ideal x','measured x','ideal z', ...
            'measured z'},'Location','best');
        title(axAccelerometer,'RMS lever rescales fixed error');

        cla(axBroken);
        plot(axBroken,out.time_s,out.idealSpecificForceBody_mps2(3,:), ...
            'LineWidth',1.7); hold(axBroken,'on');
        plot(axBroken,out.time_s, ...
            out.brokenIdealSpecificForceBody_mps2(3,:), ...
            '--','LineWidth',1.7);
        grid(axBroken,'on');
        xlabel(axBroken,'Time (s)');
        ylabel(axBroken,'Ideal body z specific force (m/s^2)');
        legend(axBroken,{'complete a-g','broken a only'}, ...
            'Location','best');
        title(axBroken,'Broken gravity omission');

        summary.Text=sprintf([ ...
            'Move one lever and reset: bias %.2f deg/s changes gyro only; accelerometer RMS %.2f m/s^2 rescales one fixed error shape only.\n' ...
            'final angle error %.2f deg = bias x %.1f s | observed accelerometer error vector RMS %.2f m/s^2\n' ...
            'supported-level complete ideal body z %.5f m/s^2 | broken %.5f m/s^2 | complete-broken magnitude error %.2g m/s^2'], ...
            out.gyroBias_deg_s,out.accelerometerNoiseRms_mps2, ...
            out.finalPitchAngleError_deg,out.timeHorizon_s, ...
            out.accelerometerNoiseVectorRmsMeasured_mps2, ...
            out.idealSpecificForceBody_mps2(3,1), ...
            out.brokenIdealSpecificForceBody_mps2(3,1), ...
            out.maxGravityOmissionMagnitudeError_mps2);
    end
end
