function plot_response(parameters)
    % plot_response  generates the graphics with the response of the
    % vehicle during the race.

    %% Response calculation
    solution = vehicle_response(parameters);
    
    %% Response data
    taVec   = solution.taVec;
    xaC     = solution.xaC;
    va      = solution.va;
    aa      = solution.aa;
    tdVec   = solution.tdVec;
    xdC     = solution.xdC;
    vd      = solution.vd;
    ad      = solution.ad;
    Delta   = solution.Delta;

    %% Graphs
    figure
    subplot(4,1,1)
        hold on ; grid on ; box on
        plot(taVec,xaC,'b')
        ylabel('Pos. [m]')
        if length(xdC) > 1
            plot(tdVec,xdC,'g')
            legend('Acc. Analytical','Dec. Analytical','Location','EastOutside')
        else
            legend('Acc. Analytical','Location','EastOutside')
        end
    subplot(4,1,2)
        hold on ; grid on ; box on
        set(gca,'ylim',[0 1.1*max(va)])
        plot(taVec,va,'b')
        ylabel('Vel. [m/s]')
        if length(xdC) > 1
            plot(tdVec,vd,'g')
            legend('Acc. Analytical','Dec. Analytical','Location','EastOutside')
        else
            legend('Acc. Analytical','Location','EastOutside')
        end
    subplot(4,1,3)
        hold on ; grid on ; box on
        plot(taVec,aa,'b')
        ylabel('Acc. [m/s2]')
        if length(xdC) > 1
            plot(tdVec,ad,'g')
            legend('Acc. Analytical','Dec. Analytical','Location','EastOutside')
            set(gca,'ylim',[1.1*min(ad) 1.1*max(aa)])
        else
            set(gca,'ylim',[0 1.1*max(aa)])
            legend('Acc. Analytical','Location','EastOutside')
        end
    subplot(4,1,4)
        hold on ; grid on ; box on
        set(gca,'ylim',[0 1.1*max(Delta)])
        plot(taVec,Delta,'b')
        xlabel('Time [s]')
        ylabel('Spring def. [m]')
        if length(xdC) > 1
            plot(tdVec,zeros(1,length(tdVec)),'g')
            legend('Acc. Analytical','Dec. Analytical','Location','EastOutside')
        else
            legend('Acc. Analytical','Location','EastOutside')
        end
    
end