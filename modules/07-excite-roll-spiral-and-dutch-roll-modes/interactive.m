function interactive
%INTERACTIVE Explore excitation and shaping of three lateral modes.
clear model;
modelFcn=@model;
existingUi=findall(groot,'Type','figure','Name', ...
    'P07 Roll, Spiral, and Dutch-Roll Modes');
if ~isempty(existingUi)
    close(existingUi);
end
fig=uifigure('Name','P07 Roll, Spiral, and Dutch-Roll Modes', ...
    'Position',[20 60 1840 800]);
gridLayout=uigridlayout(fig,[6 6]);
gridLayout.RowHeight={'1x','1x','1x',118,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x','1x','1x'};

axRollRate=uiaxes(gridLayout);
axRollRate.Layout.Row=[1 3]; axRollRate.Layout.Column=1;
axSpiralBank=uiaxes(gridLayout);
axSpiralBank.Layout.Row=[1 3]; axSpiralBank.Layout.Column=2;
axSpiralHeading=uiaxes(gridLayout);
axSpiralHeading.Layout.Row=[1 3]; axSpiralHeading.Layout.Column=3;
axDutchSideslip=uiaxes(gridLayout);
axDutchSideslip.Layout.Row=[1 3]; axDutchSideslip.Layout.Column=4;
axDutchYawRate=uiaxes(gridLayout);
axDutchYawRate.Layout.Row=[1 3]; axDutchYawRate.Layout.Column=5;
axDutchEnergy=uiaxes(gridLayout);
axDutchEnergy.Layout.Row=[1 3]; axDutchEnergy.Layout.Column=6;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 6];

aileronLabel=uilabel(gridLayout, ...
    'Text','Aileron pulse, + right roll (deg)', ...
    'HorizontalAlignment','center');
aileronLabel.Layout.Row=5; aileronLabel.Layout.Column=1;
bankLabel=uilabel(gridLayout, ...
    'Text','Initial bank release, + right wing down (deg)', ...
    'HorizontalAlignment','center');
bankLabel.Layout.Row=5; bankLabel.Layout.Column=2;
rudderLabel=uilabel(gridLayout, ...
    'Text','Rudder pulse, + nose right (deg)', ...
    'HorizontalAlignment','center');
rudderLabel.Layout.Row=5; rudderLabel.Layout.Column=3;
rollDecayLabel=uilabel(gridLayout, ...
    'Text','Roll decay rate lambda_R (1/s)', ...
    'HorizontalAlignment','center');
rollDecayLabel.Layout.Row=5; rollDecayLabel.Layout.Column=4;
spiralDecayLabel=uilabel(gridLayout, ...
    'Text','Stable spiral decay rate lambda_S (1/s)', ...
    'HorizontalAlignment','center');
spiralDecayLabel.Layout.Row=5; spiralDecayLabel.Layout.Column=5;
dutchDampingLabel=uilabel(gridLayout, ...
    'Text','Dutch-roll damping ratio zeta_D (-)', ...
    'HorizontalAlignment','center');
dutchDampingLabel.Layout.Row=5; dutchDampingLabel.Layout.Column=6;

aileronControl=uislider(gridLayout,'Limits',[-5 5],'Value',2, ...
    'MajorTicks',[-5 -2 0 2 5]);
aileronControl.Layout.Row=6; aileronControl.Layout.Column=1;
bankControl=uislider(gridLayout,'Limits',[-10 10],'Value',5, ...
    'MajorTicks',[-10 -5 0 5 10]);
bankControl.Layout.Row=6; bankControl.Layout.Column=2;
rudderControl=uislider(gridLayout,'Limits',[-5 5],'Value',3, ...
    'MajorTicks',[-5 -3 0 3 5]);
rudderControl.Layout.Row=6; rudderControl.Layout.Column=3;
rollDecayControl=uislider(gridLayout,'Limits',[0.8 5],'Value',2.5, ...
    'MajorTicks',[0.8 1.5 2.5 3.5 5]);
rollDecayControl.Layout.Row=6; rollDecayControl.Layout.Column=4;
spiralDecayControl=uislider(gridLayout,'Limits',[0 0.05],'Value',0.025, ...
    'MajorTicks',[0 0.01 0.025 0.04 0.05]);
spiralDecayControl.Layout.Row=6; spiralDecayControl.Layout.Column=5;
dutchDampingControl=uislider(gridLayout,'Limits',[0 0.6],'Value',0.18, ...
    'MajorTicks',[0 0.08 0.18 0.3 0.45 0.6]);
dutchDampingControl.Layout.Row=6; dutchDampingControl.Layout.Column=6;

aileronControl.ValueChangingFcn=@(~,event) updatePlots(event,'aileron');
bankControl.ValueChangingFcn=@(~,event) updatePlots(event,'bank');
rudderControl.ValueChangingFcn=@(~,event) updatePlots(event,'rudder');
rollDecayControl.ValueChangingFcn=@(~,event) updatePlots(event,'rollDecay');
spiralDecayControl.ValueChangingFcn=@(~,event) updatePlots(event,'spiralDecay');
dutchDampingControl.ValueChangingFcn=@(~,event) updatePlots(event,'dutchDamping');
controls=[aileronControl bankControl rudderControl rollDecayControl ...
    spiralDecayControl dutchDampingControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        aileronPulse_deg=aileronControl.Value;
        bankRelease_deg=bankControl.Value;
        rudderPulse_deg=rudderControl.Value;
        rollDecayRate_per_s=rollDecayControl.Value;
        spiralDecayRate_per_s=spiralDecayControl.Value;
        dutchRollDampingRatio=dutchDampingControl.Value;
        if nargin==2
            switch changingControl
                case 'aileron'
                    aileronPulse_deg=event.Value;
                case 'bank'
                    bankRelease_deg=event.Value;
                case 'rudder'
                    rudderPulse_deg=event.Value;
                case 'rollDecay'
                    rollDecayRate_per_s=event.Value;
                case 'spiralDecay'
                    spiralDecayRate_per_s=event.Value;
                case 'dutchDamping'
                    dutchRollDampingRatio=event.Value;
            end
        end

        out=modelFcn(aileronPulse_deg,bankRelease_deg,rudderPulse_deg, ...
            rollDecayRate_per_s,spiralDecayRate_per_s, ...
            dutchRollDampingRatio);

        cla(axRollRate);
        plot(axRollRate,out.rollTime_s,out.rollRate_deg_s,'LineWidth',1.6);
        hold(axRollRate,'on');
        plot(axRollRate,out.rollTime_s,zeros(size(out.rollTime_s)),'k--');
        grid(axRollRate,'on');
        xlabel(axRollRate,'Time after aileron pulse (s)');
        ylabel(axRollRate,'Roll rate p (deg/s)');
        title(axRollRate,'Fast roll subsidence');

        cla(axSpiralBank);
        plot(axSpiralBank,out.spiralTime_s,out.spiralBank_deg, ...
            'LineWidth',1.6); hold(axSpiralBank,'on');
        plot(axSpiralBank,out.spiralTime_s, ...
            zeros(size(out.spiralTime_s)),'k--');
        grid(axSpiralBank,'on');
        xlabel(axSpiralBank,'Time after bank release (s)');
        ylabel(axSpiralBank,'Bank angle phi (deg)');
        title(axSpiralBank,'Slow spiral mode');

        cla(axSpiralHeading);
        plot(axSpiralHeading,out.spiralTime_s, ...
            out.spiralHeadingChange_deg,'LineWidth',1.6);
        grid(axSpiralHeading,'on');
        xlabel(axSpiralHeading,'Time after bank release (s)');
        ylabel(axSpiralHeading,'Heading change (deg)');
        title(axSpiralHeading,'Small-angle turn proxy');

        cla(axDutchSideslip);
        plot(axDutchSideslip,out.dutchRollTime_s, ...
            out.dutchRollSideslip_deg,'LineWidth',1.6); hold(axDutchSideslip,'on');
        plot(axDutchSideslip,out.dutchRollTime_s, ...
            out.dutchRollSideslipEnvelope_deg,'k--');
        plot(axDutchSideslip,out.dutchRollTime_s, ...
            -out.dutchRollSideslipEnvelope_deg,'k--');
        grid(axDutchSideslip,'on');
        xlabel(axDutchSideslip,'Time after rudder pulse (s)');
        ylabel(axDutchSideslip,'Sideslip beta (deg)');
        title(axDutchSideslip,'Dutch-roll displacement');

        cla(axDutchYawRate);
        plot(axDutchYawRate,out.dutchRollTime_s, ...
            out.dutchRollYawRate_deg_s,'LineWidth',1.6); hold(axDutchYawRate,'on');
        plot(axDutchYawRate,out.dutchRollTime_s, ...
            zeros(size(out.dutchRollTime_s)),'k--');
        grid(axDutchYawRate,'on');
        xlabel(axDutchYawRate,'Time after rudder pulse (s)');
        ylabel(axDutchYawRate,'Yaw rate r (deg/s)');
        title(axDutchYawRate,'Dutch-roll rate');

        cla(axDutchEnergy);
        if out.dutchRollModalEnergy_rad2_s2(1)==0
            energyRatio=zeros(size(out.dutchRollTime_s));
        else
            energyRatio=out.dutchRollModalEnergy_rad2_s2/ ...
                out.dutchRollModalEnergy_rad2_s2(1);
        end
        plot(axDutchEnergy,out.dutchRollTime_s,energyRatio,'LineWidth',1.6);
        grid(axDutchEnergy,'on');
        xlabel(axDutchEnergy,'Time after rudder pulse (s)');
        ylabel(axDutchEnergy,'Normalized modal energy (-)');
        title(axDutchEnergy,'Energy is not physical joules');

        summary.Text=sprintf([ ...
            'Move one lever, inspect only its owned mode, explain the mechanism, then reset.  %s | %s\n' ...
            'aileron %+.2f deg -> p(0+) %+.2f deg/s | tau_R %.2f s | bank increment %+.2f deg | 2%% settle %.2f s\n' ...
            'bank release %+.2f deg -> phi(120 s) %+.2f deg | heading change %+.2f deg | fixed V %.1f m/s | lambda_S %.3f 1/s\n' ...
            'rudder %+.2f deg -> r(0+) %+.2f deg/s | peak |beta| %.2f deg | T_D %.2f s | decay/cycle %.3f\n' ...
            'Accepted inputs keep |roll-mode bank| and |spiral bank| <= %.1f deg and |beta| <= %.1f deg.'], ...
            out.spiralStabilityLabel,out.dutchRollDampingLabel, ...
            out.aileronPulse_deg,out.initialRollRate_deg_s, ...
            out.rollTimeConstant_s,out.rollAsymptoticBankChange_deg, ...
            out.rollTwoPercentSettlingTime_s,out.bankRelease_deg, ...
            out.spiralBank_deg(end),out.spiralHeadingChange_deg(end), ...
            out.referenceTrueAirspeed_mps,out.spiralDecayRate_per_s, ...
            out.rudderPulse_deg, ...
            out.initialYawRate_deg_s,out.dutchRollPeakSideslip_deg, ...
            out.dutchRollDampedPeriod_s, ...
            out.dutchRollDecayPerPeriod_ratio,out.bankLinearLimit_deg, ...
            out.sideslipLinearLimit_deg);
    end
end
