function interactive
%INTERACTIVE Explore how lateral stability derivatives create coupled motion.
clear model;
modelFcn=@model;
existingUi=findall(groot,'Type','figure','Name', ...
    'P08 Stability Derivatives to Motion');
if ~isempty(existingUi)
    close(existingUi);
end
fig=uifigure('Name','P08 Stability Derivatives to Motion', ...
    'Position',[20 60 1840 800]);
gridLayout=uigridlayout(fig,[6 6]);
gridLayout.RowHeight={'1x','1x','1x',122,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x','1x','1x'};

axSideslip=uiaxes(gridLayout);
axSideslip.Layout.Row=[1 3]; axSideslip.Layout.Column=1;
axRollRate=uiaxes(gridLayout);
axRollRate.Layout.Row=[1 3]; axRollRate.Layout.Column=2;
axYawRate=uiaxes(gridLayout);
axYawRate.Layout.Row=[1 3]; axYawRate.Layout.Column=3;
axBank=uiaxes(gridLayout);
axBank.Layout.Row=[1 3]; axBank.Layout.Column=4;
axRollLedger=uiaxes(gridLayout);
axRollLedger.Layout.Row=[1 3]; axRollLedger.Layout.Column=5;
axYawLedger=uiaxes(gridLayout);
axYawLedger.Layout.Row=[1 3]; axYawLedger.Layout.Column=6;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 6];

betaLabel=uilabel(gridLayout, ...
    'Text','Initial sideslip beta(0), + velocity right (deg)', ...
    'HorizontalAlignment','center');
betaLabel.Layout.Row=5; betaLabel.Layout.Column=[1 2];
rollDampingLabel=uilabel(gridLayout, ...
    'Text','Roll damping derivative C_l_p, acts on p-hat (-)', ...
    'HorizontalAlignment','center');
rollDampingLabel.Layout.Row=5; rollDampingLabel.Layout.Column=[3 4];
weathercockLabel=uilabel(gridLayout, ...
    'Text','Weathercock derivative C_n_beta (1/rad)', ...
    'HorizontalAlignment','center');
weathercockLabel.Layout.Row=5; weathercockLabel.Layout.Column=[5 6];

betaControl=uislider(gridLayout,'Limits',[-4 4],'Value',3, ...
    'MajorTicks',[-4 -2 0 2 4]);
betaControl.Layout.Row=6; betaControl.Layout.Column=[1 2];
rollDampingControl=uislider(gridLayout,'Limits',[-0.8 -0.3], ...
    'Value',-0.50,'MajorTicks',[-0.8 -0.65 -0.5 -0.4 -0.3]);
rollDampingControl.Layout.Row=6; rollDampingControl.Layout.Column=[3 4];
weathercockControl=uislider(gridLayout,'Limits',[0 0.24], ...
    'Value',0.18,'MajorTicks',[0 0.06 0.12 0.18 0.24]);
weathercockControl.Layout.Row=6; weathercockControl.Layout.Column=[5 6];

betaControl.ValueChangingFcn=@(~,event) updatePlots(event,'beta');
rollDampingControl.ValueChangingFcn= ...
    @(~,event) updatePlots(event,'rollDamping');
weathercockControl.ValueChangingFcn= ...
    @(~,event) updatePlots(event,'weathercock');
controls=[betaControl rollDampingControl weathercockControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        initialSideslip_deg=betaControl.Value;
        rollDampingDerivative_Cl_p=rollDampingControl.Value;
        weathercockDerivative_Cn_beta_perRad=weathercockControl.Value;
        if nargin==2
            switch changingControl
                case 'beta'
                    initialSideslip_deg=event.Value;
                case 'rollDamping'
                    rollDampingDerivative_Cl_p=event.Value;
                case 'weathercock'
                    weathercockDerivative_Cn_beta_perRad=event.Value;
            end
        end

        out=modelFcn(initialSideslip_deg,rollDampingDerivative_Cl_p, ...
            weathercockDerivative_Cn_beta_perRad);

        cla(axSideslip);
        plot(axSideslip,out.time_s,out.sideslip_deg,'LineWidth',1.6);
        hold(axSideslip,'on');
        plot(axSideslip,out.time_s,zeros(size(out.time_s)),'k--');
        grid(axSideslip,'on');
        xlabel(axSideslip,'Time after release (s)');
        ylabel(axSideslip,'Sideslip beta (deg)');
        title(axSideslip,'Lateral state');

        cla(axRollRate);
        plot(axRollRate,out.time_s,out.rollRate_deg_s,'LineWidth',1.6);
        hold(axRollRate,'on');
        plot(axRollRate,out.time_s,zeros(size(out.time_s)),'k--');
        grid(axRollRate,'on');
        xlabel(axRollRate,'Time after release (s)');
        ylabel(axRollRate,'Roll rate p (deg/s)');
        title(axRollRate,'Right-wing-down positive');

        cla(axYawRate);
        plot(axYawRate,out.time_s,out.yawRate_deg_s,'LineWidth',1.6);
        hold(axYawRate,'on');
        plot(axYawRate,out.time_s,zeros(size(out.time_s)),'k--');
        grid(axYawRate,'on');
        xlabel(axYawRate,'Time after release (s)');
        ylabel(axYawRate,'Yaw rate r (deg/s)');
        title(axYawRate,'Nose-right positive');

        cla(axBank);
        plot(axBank,out.time_s,out.bankAngle_deg,'LineWidth',1.6);
        hold(axBank,'on');
        plot(axBank,out.time_s,zeros(size(out.time_s)),'k--');
        grid(axBank,'on');
        xlabel(axBank,'Time after release (s)');
        ylabel(axBank,'Bank angle phi (deg)');
        title(axBank,'Integrated coupled bank');

        cla(axRollLedger);
        plot(axRollLedger,out.time_s,[out.rollMomentCoefficientFromBeta; ...
            out.rollMomentCoefficientFromRollRate; ...
            out.rollMomentCoefficientFromYawRate],'LineWidth',1.2);
        grid(axRollLedger,'on');
        xlabel(axRollLedger,'Time after release (s)');
        ylabel(axRollLedger,'C_l contribution (-)');
        legend(axRollLedger,{'beta term','p-hat term','r-hat term'}, ...
            'Location','best');
        title(axRollLedger,'Roll derivative ledger');

        cla(axYawLedger);
        plot(axYawLedger,out.time_s,[out.yawMomentCoefficientFromBeta; ...
            out.yawMomentCoefficientFromRollRate; ...
            out.yawMomentCoefficientFromYawRate],'LineWidth',1.2);
        grid(axYawLedger,'on');
        xlabel(axYawLedger,'Time after release (s)');
        ylabel(axYawLedger,'C_n contribution (-)');
        legend(axYawLedger,{'beta term','p-hat term','r-hat term'}, ...
            'Location','best');
        title(axYawLedger,'Yaw derivative ledger');

        if out.initialSideslip_deg==0
            crossingText='no release: every linear state remains zero';
        else
            crossingText=sprintf('first beta zero %.2f s', ...
                out.firstSideslipZeroCrossing_s);
        end
        summary.Text=sprintf([ ...
            'Move one lever, inspect its derivative ledger, then follow all four coupled states.  %s\n' ...
            'beta(0) %+.2f deg -> beta-dot(0) %+.2f deg/s | p-dot(0) %+.2f deg/s^2 | r-dot(0) %+.2f deg/s^2\n' ...
            'C_l_p %+.3f -> peak |p| %.2f deg/s | peak |phi| %.2f deg | C_n_beta %+.3f /rad -> peak |r| %.2f deg/s\n' ...
            'p-hat and r-hat use b/(2V0)=%.5f s at V0=%.1f m/s; accepted motion limits: |beta|<=%.1f deg, |phi|<=%.1f deg, rates<=%.1f deg/s.'], ...
            crossingText,out.initialSideslip_deg,out.sideslipRate_deg_s(1), ...
            out.rollAcceleration_deg_s2(1),out.yawAcceleration_deg_s2(1), ...
            out.rollDampingDerivative_Cl_p,out.peakAbsRollRate_deg_s, ...
            out.peakAbsBank_deg,out.weathercockDerivative_Cn_beta_perRad, ...
            out.peakAbsYawRate_deg_s,out.rateNormalizationTime_s, ...
            out.referenceTrueAirspeed_mps,out.sideslipLinearLimit_deg, ...
            out.bankLinearLimit_deg,out.bodyRateLearningLimit_deg_s);
    end
end
