function interactive
fig=uifigure('Name','P01 Coordinated Flight Turn','Position',[100 100 1120 720]);
g=uigridlayout(fig,[3 5]); g.RowHeight={'1x','1x',100};
axPath=uiaxes(g); axPath.Layout.Row=[1 2]; axPath.Layout.Column=[1 3];
axState=uiaxes(g); axState.Layout.Row=1; axState.Layout.Column=[4 5];
summary=uilabel(g,'WordWrap','on'); summary.Layout.Row=2; summary.Layout.Column=[4 5];

v=uislider(g,'Limits',[20 250],'Value',70,'MajorTicks',[20 50 100 150 200 250]);
v.Layout.Row=3; v.Layout.Column=1;
b=uislider(g,'Limits',[-70 70],'Value',25,'MajorTicks',[-70 -45 -20 0 20 45 70]);
b.Layout.Row=3; b.Layout.Column=2;
d=uislider(g,'Limits',[5 120],'Value',30); d.Layout.Row=3; d.Layout.Column=3;
c=uislider(g,'Limits',[-20 20],'Value',2); c.Layout.Row=3; c.Layout.Column=4;
label=uilabel(g,'Text','speed | bank | duration | climb rate','WordWrap','on');
label.Layout.Row=3; label.Layout.Column=5;
controls=[v b d c];
for i=1:numel(controls)
    controls(i).ValueChangingFcn=@(~,~) updatePlots();
    controls(i).ValueChangedFcn=@(~,~) updatePlots();
end
updatePlots();

    function updatePlots
        out=model(v.Value,b.Value,d.Value,c.Value);
        cla(axPath); plot3(axPath,out.x,out.y,out.z,'LineWidth',1.3);
        grid(axPath,'on'); axis(axPath,'equal'); xlabel(axPath,'East'); ylabel(axPath,'North');
        zlabel(axPath,'Altitude'); title(axPath,'Point-mass trajectory');

        cla(axState); yyaxis(axState,'left'); plot(axState,out.t,rad2deg(out.heading),'LineWidth',1.2);
        ylabel(axState,'Heading (deg)'); yyaxis(axState,'right');
        plot(axState,out.t,out.z,'LineWidth',1.2); ylabel(axState,'Altitude change (m)');
        grid(axState,'on'); xlabel(axState,'Time (s)'); title(axState,'State evolution');

        summary.Text=sprintf(['speed %.1f m/s\nbank %.1f deg\nradius %.1f m\n' ...
            'turn rate %.2f deg/s\nload factor %.2f g'], ...
            v.Value,b.Value,out.radius,rad2deg(out.turnRate),out.loadFactor);
    end
end
