function [angle, torque, error] = control_law(t, Y, last_error, sum_error, xx, yy)
    torque = [0 0 400 400];

    if Y(1)>=95&&Y(2)<=20
        [d, idx] = min(sqrt((xx-Y(1)).^2+(yy-Y(2)).^2));

        if xx(idx)-Y(1)>=0 && yy(idx)-Y(2)<=0
            d = -d;
        end
        if idx == length(xx)
            idx = idx-1;
        end
        error = [d, atan2(yy(idx+1)-yy(idx), xx(idx+1)-xx(idx))-Y(3)];
    elseif Y(1)<=90
        if Y(1)>70
            error = [5-Y(2),-Y(3)];
        else
            error = [-Y(2),-Y(3)];
        end
    else
        error = [-120+Y(1), pi/2-Y(3)];
    end
        
    
    
    d_error = error - last_error;

    angle = 0.8*(0.04*error(1)+0.6*error(2))+10*(0.04*d_error(1)+0.6*d_error(2));
    if abs(angle)>pi/4
        angle = sign(angle)*pi/4;
    end







end