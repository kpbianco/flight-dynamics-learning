function interactive
%INTERACTIVE Explore excitation and damping of two longitudinal modes.
clear model;
modelFcn=@model;
existingUi=findall(groot,'Type','figure','Name', ...
    'P06 Short-Period and Phugoid Modes');
if ~isempty(existingUi)
    close(existingUi);
end
fig=uifigure('Name','P06 Short-Period and Phugoid Modes', ...
    'Position',[40 60 1480 800]);
gridLayout=uigridlayout(fig,[6 4]);
gridLayout.RowHeight={'1x','1x','1x',112,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x'};

axShortAlpha=uiaxes(gridLayout);
axShortAlpha.Layout.Row=[1 3]; axShortAlpha.Layout.Column=1;
axShortRate=uiaxes(gridLayout);
axShortRate.Layout.Row=[1 3]; axShortRate.Layout.Column=2;
axPhugoidSpeed=uiaxes(gridLayout);
axPhugoidSpeed.Layout.Row=[1 3]; axPhugoidSpeed.Layout.Column=3;
axPhugoidPath=uiaxes(gridLayout);
axPhugoidPath.Layout.Row=[1 3]; axPhugoidPath.Layout.Column=4;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 4];

elevatorLabel=uilabel(gridLayout, ...
    'Text','Elevator pulse, trailing-edge down + (deg)', ...
    'HorizontalAlignment','center');
elevatorLabel.Layout.Row=5; elevatorLabel.Layout.Column=1;
airspeedLabel=uilabel(gridLayout, ...
    'Text','Initial airspeed/energy displacement (m/s)', ...
    'HorizontalAlignment','center');
airspeedLabel.Layout.Row=5; airspeedLabel.Layout.Column=2;
shortDampingLabel=uilabel(gridLayout, ...
    'Text','Short-period damping ratio zeta_sp (-)', ...
    'HorizontalAlignment','center');
shortDampingLabel.Layout.Row=5; shortDampingLabel.Layout.Column=3;
phugoidDampingLabel=uilabel(gridLayout, ...
    'Text','Phugoid damping ratio zeta_ph (-)', ...
    'HorizontalAlignment','center');
phugoidDampingLabel.Layout.Row=5; phugoidDampingLabel.Layout.Column=4;

elevatorControl=uislider(gridLayout,'Limits',[-5 5],'Value',-2, ...
    'MajorTicks',[-5 -2 0 2 5]);
elevatorControl.Layout.Row=6; elevatorControl.Layout.Column=1;
airspeedControl=uislider(gridLayout,'Limits',[-10 10],'Value',5, ...
    'MajorTicks',[-10 -5 0 5 10]);
airspeedControl.Layout.Row=6; airspeedControl.Layout.Column=2;
shortDampingControl=uislider(gridLayout,'Limits',[0 0.8],'Value',0.35, ...
    'MajorTicks',[0 0.2 0.35 0.5 0.8]);
shortDampingControl.Layout.Row=6; shortDampingControl.Layout.Column=3;
phugoidDampingControl=uislider(gridLayout,'Limits',[0 0.3],'Value',0.08, ...
    'MajorTicks',[0 0.04 0.08 0.16 0.3]);
phugoidDampingControl.Layout.Row=6; phugoidDampingControl.Layout.Column=4;

elevatorControl.ValueChangingFcn=@(~,event) updatePlots(event,'elevator');
airspeedControl.ValueChangingFcn=@(~,event) updatePlots(event,'airspeed');
shortDampingControl.ValueChangingFcn=@(~,event) updatePlots(event,'short');
phugoidDampingControl.ValueChangingFcn=@(~,event) updatePlots(event,'phugoid');
controls=[elevatorControl airspeedControl shortDampingControl ...
    phugoidDampingControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        elevatorPulse_deg=elevatorControl.Value;
        airspeedKick_mps=airspeedControl.Value;
        shortPeriodDampingRatio=shortDampingControl.Value;
        phugoidDampingRatio=phugoidDampingControl.Value;
        if nargin==2
            switch changingControl
                case 'elevator'
                    elevatorPulse_deg=event.Value;
                case 'airspeed'
                    airspeedKick_mps=event.Value;
                case 'short'
                    shortPeriodDampingRatio=event.Value;
                case 'phugoid'
                    phugoidDampingRatio=event.Value;
            end
        end

        out=modelFcn(elevatorPulse_deg,airspeedKick_mps, ...
            shortPeriodDampingRatio,phugoidDampingRatio);

        cla(axShortAlpha);
        plot(axShortAlpha,out.fastTime_s,out.shortPeriodAlpha_deg, ...
            'LineWidth',1.6); hold(axShortAlpha,'on');
        plot(axShortAlpha,out.fastTime_s,out.shortPeriodAlphaEnvelope_deg, ...
            'k--');
        plot(axShortAlpha,out.fastTime_s,-out.shortPeriodAlphaEnvelope_deg, ...
            'k--');
        grid(axShortAlpha,'on');
        xlabel(axShortAlpha,'Time after elevator pulse (s)');
        ylabel(axShortAlpha,'delta alpha (deg)');
        title(axShortAlpha,'Short-period displacement and envelope');

        cla(axShortRate);
        plot(axShortRate,out.fastTime_s,out.shortPeriodPitchRate_deg_s, ...
            'LineWidth',1.6); hold(axShortRate,'on');
        plot(axShortRate,out.fastTime_s,zeros(size(out.fastTime_s)),'k--');
        grid(axShortRate,'on');
        xlabel(axShortRate,'Time after elevator pulse (s)');
        ylabel(axShortRate,'Pitch rate q (deg/s)');
        title(axShortRate,'Fast pitch-rate response');

        cla(axPhugoidSpeed);
        plot(axPhugoidSpeed,out.slowTime_s, ...
            out.phugoidSpeedPerturbation_mps,'LineWidth',1.6); hold(axPhugoidSpeed,'on');
        plot(axPhugoidSpeed,out.slowTime_s,zeros(size(out.slowTime_s)),'k--');
        grid(axPhugoidSpeed,'on');
        xlabel(axPhugoidSpeed,'Time after airspeed kick (s)');
        ylabel(axPhugoidSpeed,'delta V (m/s)');
        title(axPhugoidSpeed,'Phugoid speed exchange');

        cla(axPhugoidPath);
        plot(axPhugoidPath,out.slowTime_s, ...
            out.phugoidFlightPathAngle_deg,'LineWidth',1.6); hold(axPhugoidPath,'on');
        plot(axPhugoidPath,out.slowTime_s,zeros(size(out.slowTime_s)),'k--');
        grid(axPhugoidPath,'on');
        xlabel(axPhugoidPath,'Time after airspeed kick (s)');
        ylabel(axPhugoidPath,'Flight-path angle gamma (deg)');
        title(axPhugoidPath,'Slow path-angle response');

        summary.Text=sprintf([ ...
            'Move one lever, inspect its owned time scale, explain the mechanism, then reset.  %s | %s\n' ...
            'elevator pulse %+.2f deg for %.2f s -> q(0) %+.2f deg/s | peak |delta alpha| %.2f deg | T_sp %.2f s | decay/cycle %.3f\n' ...
            'airspeed kick %+.2f m/s -> peak |gamma| %.2f deg | altitude range %.2f m | T_ph %.2f s | decay/cycle %.3f | T_ph/T_sp %.1f\n' ...
            'P05 restoring stiffness sets omega_sp here; positive damping is a separate P06 assumption. Normal inputs keep peak |delta alpha| <= %.1f deg.'], ...
            out.shortPeriodDampingLabel,out.phugoidDampingLabel, ...
            out.elevatorPulse_deg,out.elevatorPulseDuration_s, ...
            out.initialPitchRate_deg_s,out.shortPeriodPeakAlpha_deg, ...
            out.shortPeriodDampedPeriod_s, ...
            out.shortPeriodDecayPerPeriod_ratio,out.airspeedKick_mps, ...
            out.phugoidPeakFlightPathAngle_deg,out.phugoidAltitudeRange_m, ...
            out.phugoidDampedPeriod_s,out.phugoidDecayPerPeriod_ratio, ...
            out.modePeriodRatio,out.shortPeriodLinearAlphaLimit_deg);
    end
end
