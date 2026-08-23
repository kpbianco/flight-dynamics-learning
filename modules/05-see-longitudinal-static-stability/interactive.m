function interactive
%INTERACTIVE Explore CG, tail size, alpha, and elevator moment increments.
clear model;
modelFcn=@model;
existingUi=findall(groot,'Type','figure','Name', ...
    'P05 Longitudinal Static Stability');
if ~isempty(existingUi)
    close(existingUi);
end
fig=uifigure('Name','P05 Longitudinal Static Stability', ...
    'Position',[60 60 1360 780]);
gridLayout=uigridlayout(fig,[6 4]);
gridLayout.RowHeight={'1x','1x','1x',100,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x'};

axMoment=uiaxes(gridLayout);
axMoment.Layout.Row=[1 3]; axMoment.Layout.Column=[1 2];
axComponents=uiaxes(gridLayout);
axComponents.Layout.Row=[1 3]; axComponents.Layout.Column=3;
axCg=uiaxes(gridLayout);
axCg.Layout.Row=[1 3]; axCg.Layout.Column=4;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 4];

cgLabel=uilabel(gridLayout, ...
    'Text','CG aft of MAC leading edge (% MAC)', ...
    'HorizontalAlignment','center');
cgLabel.Layout.Row=5; cgLabel.Layout.Column=1;
tailLabel=uilabel(gridLayout, ...
    'Text','Horizontal-tail area ratio S_t/S (%)', ...
    'HorizontalAlignment','center');
tailLabel.Layout.Row=5; tailLabel.Layout.Column=2;
alphaLabel=uilabel(gridLayout, ...
    'Text','Angle-of-attack perturbation (deg)', ...
    'HorizontalAlignment','center');
alphaLabel.Layout.Row=5; alphaLabel.Layout.Column=3;
elevatorLabel=uilabel(gridLayout, ...
    'Text','Elevator perturbation, trailing-edge down + (deg)', ...
    'HorizontalAlignment','center');
elevatorLabel.Layout.Row=5; elevatorLabel.Layout.Column=4;

cgControl=uislider(gridLayout,'Limits',[15 65],'Value',30, ...
    'MajorTicks',[15 25 30 40 50 65]);
cgControl.Layout.Row=6; cgControl.Layout.Column=1;
tailControl=uislider(gridLayout,'Limits',[0 30],'Value',20, ...
    'MajorTicks',[0 5 10 20 30]);
tailControl.Layout.Row=6; tailControl.Layout.Column=2;
alphaControl=uislider(gridLayout,'Limits',[-5 5],'Value',2, ...
    'MajorTicks',[-5 -2 0 2 5]);
alphaControl.Layout.Row=6; alphaControl.Layout.Column=3;
elevatorControl=uislider(gridLayout,'Limits',[-15 15],'Value',0, ...
    'MajorTicks',[-15 -5 0 5 15]);
elevatorControl.Layout.Row=6; elevatorControl.Layout.Column=4;

cgControl.ValueChangingFcn=@(~,event) updatePlots(event,'cg');
tailControl.ValueChangingFcn=@(~,event) updatePlots(event,'tail');
alphaControl.ValueChangingFcn=@(~,event) updatePlots(event,'alpha');
elevatorControl.ValueChangingFcn=@(~,event) updatePlots(event,'elevator');
controls=[cgControl tailControl alphaControl elevatorControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        cgPosition_percentMAC=cgControl.Value;
        tailAreaRatio_percent=tailControl.Value;
        angleOfAttackPerturbation_deg=alphaControl.Value;
        elevatorPerturbation_deg=elevatorControl.Value;
        if nargin==2
            switch changingControl
                case 'cg'
                    cgPosition_percentMAC=event.Value;
                case 'tail'
                    tailAreaRatio_percent=event.Value;
                case 'alpha'
                    angleOfAttackPerturbation_deg=event.Value;
                case 'elevator'
                    elevatorPerturbation_deg=event.Value;
            end
        end

        out=modelFcn(cgPosition_percentMAC,tailAreaRatio_percent, ...
            angleOfAttackPerturbation_deg,elevatorPerturbation_deg);
        alphaGrid_deg=-5:0.5:5;
        momentGrid=zeros(size(alphaGrid_deg));
        for index=1:numel(alphaGrid_deg)
            alphaSample=modelFcn(cgPosition_percentMAC, ...
                tailAreaRatio_percent,alphaGrid_deg(index), ...
                elevatorPerturbation_deg);
            momentGrid(index)=alphaSample.pitchingMomentCoefficient;
        end
        cgGrid_percentMAC=15:2:65;
        slopeGrid_perRad=zeros(size(cgGrid_percentMAC));
        for index=1:numel(cgGrid_percentMAC)
            cgSample=modelFcn(cgGrid_percentMAC(index), ...
                tailAreaRatio_percent,0,0);
            slopeGrid_perRad(index)=cgSample.pitchingMomentSlope_perRad;
        end

        cla(axMoment);
        plot(axMoment,alphaGrid_deg,momentGrid,'LineWidth',1.6); hold(axMoment,'on');
        plot(axMoment,alphaGrid_deg,zeros(size(alphaGrid_deg)),'k--');
        plot(axMoment,out.angleOfAttackPerturbation_deg, ...
            out.pitchingMomentCoefficient,'o','MarkerSize',9,'LineWidth',2);
        grid(axMoment,'on');
        xlabel(axMoment,'Angle-of-attack perturbation, delta alpha (deg)');
        ylabel(axMoment,'Pitching-moment increment, delta C_m (-)');
        title(axMoment,'Slope sets stability; elevator shifts the intercept');
        legend(axMoment,{'selected configuration','zero moment','selected state'}, ...
            'Location','best');

        cla(axComponents);
        bar(axComponents,[out.wingMomentSlope_perRad ...
            out.tailMomentSlope_perRad out.pitchingMomentSlope_perRad]);
        grid(axComponents,'on');
        axComponents.XTick=1:3;
        axComponents.XTickLabel={'wing','tail','total'};
        ylabel(axComponents,'dC_m/dalpha (1/rad)');
        title(axComponents,'Component slope buildup');

        cla(axCg);
        plot(axCg,cgGrid_percentMAC,slopeGrid_perRad,'LineWidth',1.5); hold(axCg,'on');
        plot(axCg,cgGrid_percentMAC,zeros(size(cgGrid_percentMAC)),'k--');
        plot(axCg,out.cgPosition_percentMAC,out.pitchingMomentSlope_perRad, ...
            's','MarkerSize',9,'LineWidth',2);
        plot(axCg,out.neutralPoint_percentMAC,0,'d','MarkerSize',9,'LineWidth',2);
        grid(axCg,'on');
        xlabel(axCg,'CG position aft of MAC leading edge (% MAC)');
        ylabel(axCg,'dC_m/dalpha (1/rad)');
        title(axCg,'CG relative to the neutral point');
        legend(axCg,{'slope versus CG','neutral slope','selected CG','h_n'}, ...
            'Location','best');

        summary.Text=sprintf([ ...
            'Move one lever, explain the changed view, then reset.  %s\n' ...
            'cg %.2f%% MAC | S_t/S %.2f%% | h_n %.2f%% MAC | static margin %+.2f%% MAC | C_m_alpha %+.3f /rad\n' ...
            'delta alpha %+.2f deg | absolute alpha %.2f deg | delta elevator %+.2f deg | delta C_m %+.4f | delta M %+.1f N*m\n' ...
            'Positive elevator is trailing-edge down. Elevator changes moment input, not the stick-fixed stability slope.'], ...
            out.stabilityLabel,out.cgPosition_percentMAC, ...
            out.tailAreaRatio_percent,out.neutralPoint_percentMAC, ...
            out.staticMargin_percentMAC,out.pitchingMomentSlope_perRad, ...
            out.angleOfAttackPerturbation_deg,out.absoluteAngleOfAttack_deg, ...
            out.elevatorPerturbation_deg,out.pitchingMomentCoefficient, ...
            out.pitchingMoment_Nm);
    end
end
