classdef Vehicle < handle

    properties
        a
        b
        c
        mu1
        mu2
        ks
        r
        mass
        I
    end
    
    properties (Access = private)
        position 
        rotation 
        omega    
        velocity 
        torque
        angle
        error
        sum_error
        xx
        yy
        
    end
    
    methods
        function this = Vehicle(a,b,c,mu1,mu2,ks,r,mass,I)
            this.a = a;
            this.b = b;
            this.c = c;
            this.mu1 = mu1;
            this.mu2 = mu2;
            this.ks = ks;
            this.r = r;
            this.mass = mass;
            this.angle = 0;
            this.I = I;
            this.error = [0, 0];
            this.sum_error = [0, 0];
            delta = -pi/2:0.01:0;
            this.xx = [0:0.01:100, 100+20*cos(delta), 120+zeros(size(0:0.01:100))];
            this.yy = [zeros(size(0:0.01:100)), 20+20*sin(delta), 20:0.01:120];
        end
        
        
        % ode solvers
        function [T,Y] = motion(this,t_end,ic,options)
            % this integrate the motion according to equations of motion
            [T,Y] = ode45(@(t,y) equationOfMotion(t,y,this), [0:0.0001:t_end], ic, options);

        end
        
        % odes
        function dy = equationOfMotion(t,y,this)
            theta = y(3);
            R = [cos(theta), -sin(theta), 0;
                 sin(theta),  cos(theta), 0;
                          0,           0, 1];
            yb_dot = inv(R)*[y(4);y(5);y(6)];
            [this.angle, this.torque, this.error] = control_law(t, y, this.error, this.sum_error,this.xx,this.yy);
            this.sum_error = this.sum_error + this.error;
            F = [this.torque(1) 0;
                 this.torque(2) 0;
                 this.torque(3) 0;
                 this.torque(4) 0]/this.r;
            
            v1 = [yb_dot(1)-this.c*yb_dot(3); yb_dot(2)+this.b*yb_dot(3)];
            v2 = [yb_dot(1)+this.c*yb_dot(3); yb_dot(2)+this.b*yb_dot(3)];
            v3 = [yb_dot(1)+this.c*yb_dot(3); yb_dot(2)-this.b*yb_dot(3)];
            v4 = [yb_dot(1)-this.c*yb_dot(3); yb_dot(2)-this.b*yb_dot(3)];
            
            mu = [this.mu1, this.mu2;
                  this.mu1, this.mu2;
                  this.mu1, this.mu2;
                  this.mu1, this.mu2];
              
%             %------ Full brake, do noy use it when control  --------
%             if t>1
%                 mu = [20*this.mu2, this.mu2;
%                   this.mu2, this.mu2;
%                   this.mu2, this.mu2;
%                   this.mu2, this.mu2];
%                     
%                 if t<1.2
%                     mu = [20*this.mu2, this.mu2;
%                   this.mu2, this.mu2;
%                   this.mu2, this.mu2;
%                   this.mu2, this.mu2];
%                     this.angle = pi/6;   
%                 end
%             end
            
            %---------------No brake------------------------
            if t>1
                mu = [20*this.mu1, this.mu2;
                  this.mu1, this.mu2;
                  this.mu1, this.mu2;
                  this.mu1, this.mu2];
                    
                if t<1.2
                    mu = [20*this.mu1, this.mu2;
                  this.mu1, this.mu2;
                  this.mu1, this.mu2;
                  this.mu1, this.mu2];
                    this.angle = pi/6;   
                end
            end
            
            
            
            
            
            R_wheel  = [cos(this.angle), -sin(this.angle);
                        sin(this.angle),  cos(this.angle)];
            v1_local = inv(R_wheel)*v1;
            v2_local = inv(R_wheel)*v2;
            f_local = -this.mass*9.8*2/pi*[mu(1,1)*atan(this.ks*(v1_local(1))), mu(1,2)*atan(this.ks*(v1_local(2)));
                                           mu(2,1)*atan(this.ks*(v2_local(1))), mu(2,2)*atan(this.ks*(v2_local(2)));
                                           mu(3,1)*atan(this.ks*(v3(1))), mu(3,2)*atan(this.ks*(v3(2)));
                                           mu(4,1)*atan(this.ks*(v4(1))), mu(4,2)*atan(this.ks*(v4(2)))];

                                  
            totalF = F+f_local;
                  
            realF = [(R_wheel*totalF(1,:).').';
                     (R_wheel*totalF(2,:).').';
                     totalF(3,:);
                     totalF(4,:)];
            ax = (sum(realF(:,1)))/this.mass;
            ay = (sum(realF(:,2)))/this.mass;
            next_omega = ((realF(2,1)+realF(3,1)-realF(1,1)-realF(4,1))*this.c+(realF(1,2)+realF(2,2))*this.b-(realF(3,2)+realF(4,2))*this.a)/this.I;
            
            
            state_dotdot = R*[ax-yb_dot(2)*y(6);ay+yb_dot(1)*y(6);next_omega];
                        
            dy(1,1) = y(4);
            dy(2,1) = y(5);
            dy(3,1) = y(6);
            dy(4,1) = state_dotdot(1);
            dy(5,1) = state_dotdot(2);
            dy(6,1) = state_dotdot(3);

            

        end

    end
    
end

