function interactive
%INTERACTIVE Explore altitude, temperature offset, and true airspeed.
clear model;
modelFcn=@model;
fig=uifigure('Name','P03 Atmosphere Model','Position',[80 80 1240 760]);
gridLayout=uigridlayout(fig,[6 3]);
gridLayout.RowHeight={'1x','1x','1x',82,24,58};
gridLayout.ColumnWidth={'1x','1x','1x'};

axDensity=uiaxes(gridLayout);
axDensity.Layout.Row=[1 3]; axDensity.Layout.Column=1;
axTemperature=uiaxes(gridLayout);
axTemperature.Layout.Row=[1 3]; axTemperature.Layout.Column=2;
axDynamicPressure=uiaxes(gridLayout);
axDynamicPressure.Layout.Row=[1 3]; axDynamicPressure.Layout.Column=3;

summary=uilabel(gridLayout,'WordWrap','on','HorizontalAlignment','center');
summary.Layout.Row=4; summary.Layout.Column=[1 3];

altitudeLabel=uilabel(gridLayout, ...
    'Text','Pressure altitude h, positive up (m)','HorizontalAlignment','center');
altitudeLabel.Layout.Row=5; altitudeLabel.Layout.Column=1;
temperatureLabel=uilabel(gridLayout, ...
    'Text','Local temperature offset (K)','HorizontalAlignment','center');
temperatureLabel.Layout.Row=5; temperatureLabel.Layout.Column=2;
airspeedLabel=uilabel(gridLayout, ...
    'Text','P02 true airspeed magnitude (m/s)','HorizontalAlignment','center');
airspeedLabel.Layout.Row=5; airspeedLabel.Layout.Column=3;

altitudeControl=uislider(gridLayout,'Limits',[0 20000],'Value',5000, ...
    'MajorTicks',[0 5000 11000 15000 20000]);
altitudeControl.Layout.Row=6; altitudeControl.Layout.Column=1;
temperatureControl=uislider(gridLayout,'Limits',[-40 40],'Value',0, ...
    'MajorTicks',[-40 -20 0 20 40]);
temperatureControl.Layout.Row=6; temperatureControl.Layout.Column=2;
airspeedControl=uislider(gridLayout,'Limits',[0 350],'Value',150, ...
    'MajorTicks',[0 50 150 250 350]);
airspeedControl.Layout.Row=6; airspeedControl.Layout.Column=3;

altitudeControl.ValueChangingFcn=@(~,event) updatePlots(event,'altitude');
temperatureControl.ValueChangingFcn=@(~,event) updatePlots(event,'temperature');
airspeedControl.ValueChangingFcn=@(~,event) updatePlots(event,'airspeed');
controls=[altitudeControl temperatureControl airspeedControl];
for k=1:numel(controls)
    controls(k).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots(event,changingControl)
        pressureAltitude_m=altitudeControl.Value;
        temperatureOffset_K=temperatureControl.Value;
        trueAirspeed_mps=airspeedControl.Value;
        if nargin==2
            switch changingControl
                case 'altitude'
                    pressureAltitude_m=event.Value;
                case 'temperature'
                    temperatureOffset_K=event.Value;
                case 'airspeed'
                    trueAirspeed_mps=event.Value;
            end
        end

        out=modelFcn(pressureAltitude_m,temperatureOffset_K,trueAirspeed_mps);
        altitudeGrid_m=0:1000:20000;
        densityGrid_kgpm3=zeros(size(altitudeGrid_m));
        temperatureGrid_K=zeros(size(altitudeGrid_m));
        for index=1:numel(altitudeGrid_m)
            profileSample=modelFcn(altitudeGrid_m(index),0, ...
                trueAirspeed_mps);
            densityGrid_kgpm3(index)=profileSample.density_kgpm3;
            temperatureGrid_K(index)=profileSample.temperature_K;
        end
        speedGrid_mps=0:25:350;
        dynamicPressureGrid_kPa=0.5*out.density_kgpm3*speedGrid_mps.^2/1000;

        cla(axDensity);
        plot(axDensity,densityGrid_kgpm3,altitudeGrid_m/1000, ...
            'LineWidth',1.4,'DisplayName','standard 0 K-offset column'); hold(axDensity,'on');
        plot(axDensity,out.density_kgpm3,out.pressureAltitude_m/1000,'o', ...
            'MarkerSize',9,'LineWidth',2,'DisplayName','selected local state');
        grid(axDensity,'on'); xlabel(axDensity,'Density (kg/m^3)');
        ylabel(axDensity,'Pressure altitude (km)');
        title(axDensity,'Standard profile and selected local state');
        legend(axDensity,'Location','best');

        cla(axTemperature);
        plot(axTemperature,temperatureGrid_K,altitudeGrid_m/1000, ...
            'LineWidth',1.4,'DisplayName','standard 0 K-offset column'); hold(axTemperature,'on');
        plot(axTemperature,out.temperature_K,out.pressureAltitude_m/1000,'o', ...
            'MarkerSize',9,'LineWidth',2,'DisplayName','selected local state');
        grid(axTemperature,'on'); xlabel(axTemperature,'Temperature (K)');
        ylabel(axTemperature,'Pressure altitude (km)');
        title(axTemperature,'Standard layers and selected local state');
        legend(axTemperature,'Location','best');

        cla(axDynamicPressure);
        plot(axDynamicPressure,speedGrid_mps,dynamicPressureGrid_kPa, ...
            'LineWidth',1.4); hold(axDynamicPressure,'on');
        plot(axDynamicPressure,out.trueAirspeed_mps,out.dynamicPressure_Pa/1000, ...
            'o','MarkerSize',9,'LineWidth',2);
        grid(axDynamicPressure,'on'); xlabel(axDynamicPressure,'True airspeed (m/s)');
        ylabel(axDynamicPressure,'Dynamic pressure q (kPa)');
        title(axDynamicPressure,'q = 0.5 rho V^2 at selected air state');

        summary.Text=sprintf([ ...
            'Move one lever, explain the changed view, then reset.  h %.0f m | Delta T %+.1f K | %s\n' ...
            'T %.2f K | p %.2f kPa | rho %.4f kg/m^3 | a %.2f m/s\n' ...
            'TAS %.1f m/s | Mach %.3f | q %.2f kPa | EAS %.1f m/s'], ...
            out.pressureAltitude_m,out.temperatureOffset_K,out.layer, ...
            out.temperature_K,out.pressure_Pa/1000,out.density_kgpm3, ...
            out.speedOfSound_mps,out.trueAirspeed_mps,out.mach, ...
            out.dynamicPressure_Pa/1000,out.equivalentAirspeed_mps);
    end
end
