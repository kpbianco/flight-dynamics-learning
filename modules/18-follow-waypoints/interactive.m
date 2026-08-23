function interactive
%INTERACTIVE Explore P18 waypoint arrival and course-response levers.
% Move one lever, inspect one transition, then use the exact reset.
clear model;
modelFcn=@model;
uiName='P18 Waypoint Following Explorer';
existingUI=findall(groot,'Type','figure','Name',uiName);
if ~isempty(existingUI)
    close(existingUI);
end

fig=uifigure('Name',uiName,'Position',[70 35 1320 930]);
layout=uigridlayout(fig,[6 2]);
layout.RowHeight={72,72,64,36,'1x',155};
layout.ColumnWidth={'1x','1x'};

radiusPanel=uipanel(layout,'Title','Lever 1 - waypoint arrival radius');
radiusPanel.Layout.Row=1;
radiusPanel.Layout.Column=[1 2];
radiusGrid=uigridlayout(radiusPanel,[2 2]);
radiusGrid.RowHeight={22,'1x'};
radiusGrid.ColumnWidth={300,'1x'};
uilabel(radiusGrid,'Text','Inclusive 2-D arrival radius (m)');
radiusValue=uilabel(radiusGrid,'Text','30.0 m');
radiusValue.Layout.Row=2;
radiusValue.Layout.Column=1;
radiusControl=uislider(radiusGrid,'Limits',[10 80], ...
    'Value',30,'MajorTicks',[10 20 30 50 80]);
radiusControl.Layout.Row=[1 2];
radiusControl.Layout.Column=2;

gainPanel=uipanel(layout,'Title','Lever 2 - course-response gain');
gainPanel.Layout.Row=2;
gainPanel.Layout.Column=[1 2];
gainGrid=uigridlayout(gainPanel,[2 2]);
gainGrid.RowHeight={22,'1x'};
gainGrid.ColumnWidth={300,'1x'};
uilabel(gainGrid,'Text','Course-error response gain (1/s)');
gainValue=uilabel(gainGrid,'Text','0.80 1/s');
gainValue.Layout.Row=2;
gainValue.Layout.Column=1;
gainControl=uislider(gainGrid,'Limits',[0 1.2], ...
    'Value',0.8,'MajorTicks',[0 0.2 0.4 0.8 1.2]);
gainControl.Layout.Row=[1 2];
gainControl.Layout.Column=2;

modePanel=uipanel(layout,'Title','Bearing convention and broken case');
modePanel.Layout.Row=3;
modePanel.Layout.Column=[1 2];
modeGrid=uigridlayout(modePanel,[1 3]);
modeGrid.ColumnWidth={170,350,'1x'};
uilabel(modeGrid,'Text','Bearing calculation:');
modeControl=uidropdown(modeGrid,'Items',{ ...
    'Correct atan2(East,North)', ...
    'BROKEN atan2(North,East)'}, ...
    'Value','Correct atan2(East,North)');
modeValue=uilabel(modeGrid, ...
    'Text','Correct: course is clockwise from North');

resetControl=uibutton(layout,'push', ...
    'Text',['Reset: radius 30 m, response gain 0.80 1/s, ' ...
    'correct N/E bearing']);
resetControl.Layout.Row=4;
resetControl.Layout.Column=[1 2];

plotGrid=uigridlayout(layout,[2 2]);
plotGrid.Layout.Row=5;
plotGrid.Layout.Column=[1 2];
axRoute=uiaxes(plotGrid);
axCourse=uiaxes(plotGrid);
axRange=uiaxes(plotGrid);
axRate=uiaxes(plotGrid);

summary=uilabel(layout,'Text','','WordWrap','on', ...
    'HorizontalAlignment','left','VerticalAlignment','center');
summary.Layout.Row=6;
summary.Layout.Column=[1 2];

radiusControl.ValueChangingFcn= ...
    @(source,event) updatePlots(event,'radius');
radiusControl.ValueChangedFcn= ...
    @(source,event) updatePlots(event,'radius');
gainControl.ValueChangingFcn= ...
    @(source,event) updatePlots(event,'gain');
gainControl.ValueChangedFcn= ...
    @(source,event) updatePlots(event,'gain');
modeControl.ValueChangedFcn=@(source,event) updatePlots();
resetControl.ButtonPushedFcn=@(source,event) resetBaseline();
updatePlots();

    function resetBaseline
        radiusControl.Value=30;
        gainControl.Value=0.8;
        modeControl.Value='Correct atan2(East,North)';
        updatePlots();
    end

    function updatePlots(event,changingControl)
        arrivalRadius_m=radiusControl.Value;
        courseResponseGain_per_s=gainControl.Value;
        if nargin==2
            switch changingControl
                case 'radius'
                    arrivalRadius_m=event.Value;
                case 'gain'
                    courseResponseGain_per_s=event.Value;
            end
        end

        if strcmp(modeControl.Value,'Correct atan2(East,North)')
            bearingMode=1;
            modeValue.Text= ...
                'Correct: atan2(Delta East,Delta North), clockwise from North';
        else
            bearingMode=-1;
            modeValue.Text= ...
                'Broken: a due-North target commands +90 deg East';
        end

        out=modelFcn(arrivalRadius_m,courseResponseGain_per_s,bearingMode);
        correct=modelFcn(arrivalRadius_m,courseResponseGain_per_s,1);
        radiusValue.Text=sprintf('%.1f m',out.arrivalRadius_m);
        gainValue.Text=sprintf('%.2f 1/s',out.courseResponseGain_per_s);
        moving=out.motionActive;

        cla(axRoute);
        plot(axRoute,out.waypointEast_m,out.waypointNorth_m,'k--o', ...
            'LineWidth',1.3,'MarkerFaceColor','k');
        hold(axRoute,'on');
        plot(axRoute,out.eastPosition_m(moving), ...
            out.northPosition_m(moving),'LineWidth',1.7);
        routeLegend={'planned legs','selected path'};
        if bearingMode==-1
            plot(axRoute,correct.eastPosition_m(correct.motionActive), ...
                correct.northPosition_m(correct.motionActive),':', ...
                'LineWidth',1.5);
            routeLegend{end+1}='correct comparison';
        end
        grid(axRoute,'on'); axis(axRoute,'equal');
        xlabel(axRoute,'East position (m)');
        ylabel(axRoute,'North position (m)');
        legend(axRoute,routeLegend,'Location','best');
        title(axRoute,'Fixed route and flown ground track');

        cla(axCourse);
        plot(axCourse,out.time_s(moving), ...
            out.courseCommand_deg(moving),'LineWidth',1.5);
        hold(axCourse,'on');
        plot(axCourse,out.time_s(moving),out.course_deg(moving), ...
            'LineWidth',1.5);
        grid(axCourse,'on');
        xlabel(axCourse,'Time (s)'); ylabel(axCourse,'Course (deg)');
        legend(axCourse,{'active-waypoint bearing','actual course'}, ...
            'Location','best');
        title(axCourse,'Bearing command and bounded response');

        cla(axRange);
        plot(axRange,out.time_s(moving), ...
            out.rangeToActiveWaypoint_m(moving),'LineWidth',1.5);
        hold(axRange,'on');
        plot(axRange,out.time_s(moving), ...
            out.arrivalRadius_m*ones(1,sum(moving)),'k--');
        stairs(axRange,out.time_s(moving), ...
            100*out.activeWaypointIndex(moving),':','LineWidth',1.2);
        grid(axRange,'on');
        xlabel(axRange,'Time (s)');
        ylabel(axRange,'Range (m) or 100 x waypoint index');
        legend(axRange,{'active-waypoint range','arrival radius', ...
            '100 x active index'},'Location','best');
        title(axRange,'Inclusive arrival and ordered sequencing');

        cla(axRate);
        plot(axRate,out.time_s(moving), ...
            out.courseError_deg(moving),'LineWidth',1.4);
        hold(axRate,'on');
        plot(axRate,out.time_s(moving), ...
            out.courseRateCommandUnclamped_degps(moving),'--', ...
            'LineWidth',1.2);
        plot(axRate,out.time_s(moving), ...
            out.courseRate_degps(moving),'LineWidth',1.5);
        grid(axRate,'on');
        xlabel(axRate,'Time (s)');
        ylabel(axRate,'Error (deg) or rate (deg/s)');
        legend(axRate,{'shortest course error','unclamped rate', ...
            'bounded rate'},'Location','best');
        title(axRate,'Response gain versus fixed +/-12 deg/s authority');

        if out.routeComplete
            completionText=sprintf('complete at %.1f s', ...
                out.missionCompletionTime_s);
        else
            completionText='not complete within the fixed 100 s horizon';
        end
        summary.Text=sprintf([ ...
            'Move one lever, then reset: radius %.1f m | gain %.2f 1/s | %s\n' ...
            '%s | captured targets %d/%d | flown distance %.1f m | final-target range %.1f m\n' ...
            'course-error RMS %.2f deg | peak error %.2f deg | rate saturation %.1f%% | cross-track RMS %.1f m\n' ...
            'P17 estimate is an ideal N/E input here; no P17 arrays, moving target, pursuit, intercept, lead, wind, bank, or aircraft model is used.'], ...
            out.arrivalRadius_m,out.courseResponseGain_per_s, ...
            out.bearingModeName,completionText, ...
            out.targetWaypointCapturedCount,out.waypointCount-1, ...
            out.flownDistance_m,out.finalTargetDistance_m, ...
            out.courseErrorRMS_deg,out.peakAbsoluteCourseError_deg, ...
            100*out.courseRateSaturationFraction,out.crossTrackRMS_m);
    end
end
