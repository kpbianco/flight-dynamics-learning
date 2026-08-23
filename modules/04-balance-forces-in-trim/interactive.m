function interactive
%INTERACTIVE Explore density, airspeed, mass, and flight-path force trim.
clear model;
modelFcn=@model;
fig=uifigure('Name','P04 Force Trim','Position',[60 60 1320 780]);
gridLayout=uigridlayout(fig,[6 4]);
gridLayout.RowHeight={'1x','1x','1x',92,24,58};
gridLayout.ColumnWidth={'1x','1x','1x','1x'};

axForces=uiaxes(gridLayout);
axForces.Layout.Row=[1 3]; axForces.Layout.Column=1;
axDrag=uiaxes(gridLayout);
axDrag.Layout.Row=[1 3]; axDrag.Layout.Column=[2 3];
axMargins=uiaxes(gridLayout);
axMargins.Layout.Row=[1 3]; axMargins.Layout.Column=4;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 4];

densityLabel=uilabel(gridLayout, ...
    'Text','P03 air density rho (kg/m^3)','HorizontalAlignment','center');
densityLabel.Layout.Row=5; densityLabel.Layout.Column=1;
airspeedLabel=uilabel(gridLayout, ...
    'Text','True airspeed V (m/s)','HorizontalAlignment','center');
airspeedLabel.Layout.Row=5; airspeedLabel.Layout.Column=2;
massLabel=uilabel(gridLayout, ...
    'Text','Aircraft mass (kg)','HorizontalAlignment','center');
massLabel.Layout.Row=5; massLabel.Layout.Column=3;
pathLabel=uilabel(gridLayout, ...
    'Text','Flight-path angle gamma (deg)','HorizontalAlignment','center');
pathLabel.Layout.Row=5; pathLabel.Layout.Column=4;

densityControl=uislider(gridLayout,'Limits',[0.1 1.3],'Value',0.736115547399152, ...
    'MajorTicks',[0.1 0.4 0.7361155474 1.0 1.3]);
densityControl.Layout.Row=6; densityControl.Layout.Column=1;
airspeedControl=uislider(gridLayout,'Limits',[25 150],'Value',60, ...
    'MajorTicks',[25 40 60 100 150]);
airspeedControl.Layout.Row=6; airspeedControl.Layout.Column=2;
massControl=uislider(gridLayout,'Limits',[600 2000],'Value',1200, ...
    'MajorTicks',[600 900 1200 1600 2000]);
massControl.Layout.Row=6; massControl.Layout.Column=3;
pathControl=uislider(gridLayout,'Limits',[-8 8],'Value',0, ...
    'MajorTicks',[-8 -4 0 4 8]);
pathControl.Layout.Row=6; pathControl.Layout.Column=4;

densityControl.ValueChangingFcn=@(~,event) updatePlots(event,'density');
airspeedControl.ValueChangingFcn=@(~,event) updatePlots(event,'airspeed');
massControl.ValueChangingFcn=@(~,event) updatePlots(event,'mass');
pathControl.ValueChangingFcn=@(~,event) updatePlots(event,'path');
controls=[densityControl airspeedControl massControl pathControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        airDensity_kgpm3=densityControl.Value;
        trueAirspeed_mps=airspeedControl.Value;
        mass_kg=massControl.Value;
        flightPathAngle_deg=pathControl.Value;
        if nargin==2
            switch changingControl
                case 'density'
                    airDensity_kgpm3=event.Value;
                case 'airspeed'
                    trueAirspeed_mps=event.Value;
                case 'mass'
                    mass_kg=event.Value;
                case 'path'
                    flightPathAngle_deg=event.Value;
            end
        end

        out=modelFcn(airDensity_kgpm3,trueAirspeed_mps,mass_kg, ...
            flightPathAngle_deg);
        speedGrid_mps=20:5:160;
        parasiteGrid_N=zeros(size(speedGrid_mps));
        inducedGrid_N=zeros(size(speedGrid_mps));
        thrustGrid_N=zeros(size(speedGrid_mps));
        liftCoefficientGrid=zeros(size(speedGrid_mps));
        thrustFractionGrid=zeros(size(speedGrid_mps));
        for index=1:numel(speedGrid_mps)
            speedSample=modelFcn(airDensity_kgpm3,speedGrid_mps(index), ...
                mass_kg,flightPathAngle_deg);
            parasiteGrid_N(index)=speedSample.parasiteDrag_N;
            inducedGrid_N(index)=speedSample.inducedDrag_N;
            thrustGrid_N(index)=speedSample.thrustRequired_N;
            liftCoefficientGrid(index)=speedSample.liftCoefficient;
            thrustFractionGrid(index)=speedSample.requiredThrustFraction;
        end

        cla(axForces);
        forcePairs_N=[out.lift_N -out.normalForceRequired_N; ...
            out.thrustRequired_N -(out.drag_N+out.weightAlongPath_N)];
        bar(axForces,forcePairs_N); grid(axForces,'on');
        axForces.XTick=[1 2];
        axForces.XTickLabel={'normal','along path'};
        ylabel(axForces,'Force (N)');
        title(axForces,'Required and balancing forces');
        legend(axForces,{'positive-axis force','opposing force'},'Location','best');

        cla(axDrag);
        plot(axDrag,speedGrid_mps,parasiteGrid_N,'LineWidth',1.4); hold(axDrag,'on');
        plot(axDrag,speedGrid_mps,inducedGrid_N,'LineWidth',1.4);
        plot(axDrag,speedGrid_mps,thrustGrid_N,'LineWidth',1.6);
        plot(axDrag,out.trueAirspeed_mps,out.thrustRequired_N,'o', ...
            'MarkerSize',9,'LineWidth',2);
        grid(axDrag,'on'); xlabel(axDrag,'True airspeed (m/s)');
        ylabel(axDrag,'Force (N)');
        title(axDrag,'Drag trade and required thrust');
        legend(axDrag,{'parasite drag','induced drag','required thrust','selected'}, ...
            'Location','best');

        yyaxis(axMargins,'left');
        cla(axMargins);
        yyaxis(axMargins,'right');
        cla(axMargins);
        yyaxis(axMargins,'left');
        plot(axMargins,speedGrid_mps,liftCoefficientGrid,'LineWidth',1.4, ...
            'DisplayName','required C_L'); hold(axMargins,'on');
        plot(axMargins,speedGrid_mps,out.maximumLiftCoefficient* ...
            ones(size(speedGrid_mps)),'--','LineWidth',1.2, ...
            'DisplayName','C_L max');
        plot(axMargins,out.trueAirspeed_mps,out.liftCoefficient,'o', ...
            'MarkerSize',8,'LineWidth',2,'DisplayName','selected C_L');
        ylabel(axMargins,'Lift coefficient C_L (-)');
        yyaxis(axMargins,'right');
        plot(axMargins,speedGrid_mps,thrustFractionGrid,'LineWidth',1.4, ...
            'DisplayName','required thrust/cap');
        plot(axMargins,speedGrid_mps,ones(size(speedGrid_mps)),'--','LineWidth',1.2, ...
            'DisplayName','thrust cap');
        plot(axMargins,out.trueAirspeed_mps,out.requiredThrustFraction,'s', ...
            'MarkerSize',8,'LineWidth',2,'DisplayName','selected thrust/cap');
        ylabel(axMargins,'Required thrust / maximum thrust (-)');
        grid(axMargins,'on'); xlabel(axMargins,'True airspeed (m/s)');
        title(axMargins,'Lift and thrust feasibility');
        legend(axMargins,'Location','best');

        if out.trimFeasible
            feasibility='feasible within the declared lift and thrust limits';
        elseif ~out.liftFeasible && ~out.thrustFeasible
            feasibility='not feasible: lift and thrust limits are both violated';
        elseif ~out.liftFeasible
            feasibility='not feasible: required C_L exceeds C_L max';
        elseif out.thrustRequired_N<0
            feasibility='not feasible: steady balance would require negative thrust';
        else
            feasibility='not feasible: required thrust exceeds the declared maximum';
        end
        summary.Text=sprintf([ ...
            'Move one lever, explain the changed view, then reset.  %s\n' ...
            'rho %.4f kg/m^3 | V %.1f m/s | mass %.0f kg | gamma %+.1f deg | q %.1f Pa\n' ...
            'C_L %.3f | alpha %.2f deg | lift %.1f N | drag %.1f N | thrust %.1f N | residual %.3g N\n' ...
            'stall %.1f m/s | minimum drag %.1f m/s | required thrust/cap %.1f%%'], ...
            feasibility,out.airDensity_kgpm3,out.trueAirspeed_mps,out.mass_kg, ...
            out.flightPathAngle_deg,out.dynamicPressure_Pa,out.liftCoefficient, ...
            out.angleOfAttack_deg,out.lift_N,out.drag_N,out.thrustRequired_N, ...
            out.forceResidualMagnitude_N,out.stallSpeed_mps, ...
            out.minimumDragSpeed_mps,100*out.requiredThrustFraction);
    end
end
