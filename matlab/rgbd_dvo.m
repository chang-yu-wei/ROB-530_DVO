classdef rgbd_dvo < handle
    % RGBD_DVO Summary of this class goes here
    %   Detailed explanation goes here
    
    % PUBLIC PROPERTIES
    properties
        fixed_pc;                  % fixed (target) point cloud
        moving_pc;                 % moving (source) point cloud
        fixed_image;            % fixed frame intensity image
        moving_image;           % moving frame intensity image
        imgrad;                 % image intensity gradient
        gradI;                  % intensity gradient
        MAX_ITER = 2000;        % maximum number of iteration
        % The program stops if norm(omega)+norm(v) < eps
        eps = 5*1e-5;
        eps_2 = 1e-5;
        R = eye(3);             % initial orientation 
        T = zeros(3,1);         % initial translation
        omega;                  % so(3) part of twist
        v;                      % R^3 part of twist
        tform;                  % SE(3) tf
        iterations;             % number if iterations performed 
        ids;                    % nonzero indicies of coefficient matrix A
        min_step = 2*1e-1;      % minimum step size foe integration
        step;                   % integration step
        fixed_pc_loc;                % target points
        moving_pc_loc;                % source points
        fixed_intensity;            % target intensity
        moving_intensity;            % source intensity
        U;                      % first reprojected coordinates
        V;                      % second reprojected coordinates
        valid_proj_idx;         % valid project points index in row major 
        J;                      % Jacobian
        u_max;                  % maximum horizontal image size
        v_max;                  % maximum vertical image size
        invalid_points;         % invalid points after projection
        residual;
        alpha;                  % Cauchy loss parameter
        
        % intrinsic matrix 
        fx;
        fy;
        cx;
        cy;
    end
    
    
    methods(Static)
        function Ti = tf_inv(R, t)
            % SE(3) inverse
            Ti = [R', -R' * t; 0, 0, 0, 1];
        end
        
        function d = dist_se3(R, t)
            % se(3) matrix norm

            d = norm(logm([R, t; 0, 0, 0, 1]),'fro');
        end
       
        function omega_hat = hat(omega)
            % The 'hat' function, R^3\to\Skew_3
            omega_hat = [0,         -omega(3),  omega(2);...
                        omega(3),   0,          -omega(1);...
                        -omega(2),  omega(1),   0];
        end
        
        function intensity = rgb2intensity(rgb)
            % convert RGB values to grayscale using MATLAB's rgb2gray
            % coefficients.
            try
           intensity = 0.2989 * rgb(:,1) + 0.5870 * rgb(:,2) + 0.1140 * rgb(:,3);
           if any(intensity > 1)
               intensity = double(intensity) / 255;
           end
            catch
                keyboard;
            end
        end
    end
    
    
    methods
        function obj = rgbd_dvo(varargin)
            % RGBD_DVO Construct an instance of this class
            if nargin == 2
                disp('Initial transformation is set.');
                obj.R = varargin{1};
                obj.T = varargin{2};
            elseif nargin > 0
                warning('The inputs are ignored!');
            end
        end
        
        function set_camera_intrinsic(obj, fx, fy, cx, cy)
            obj.fx = fx;
            obj.fy = fy;
            obj.cx = cx;
            obj.cy = cy;
        end
            
        
        function set_ptclouds(obj, target, source, varargin)
            % traget is fixed source is moving
            if nargin == 3
                obj.fixed_pc = ptcloud_edge_filter(target.ptcloud);
                obj.fixed_pc_loc = double(obj.fixed_pc.Location);
                obj.moving_pc = ptcloud_edge_filter(source.ptcloud);
                
                if any(target.image > 1)
                    target.image = double(target.image) / 255;
                end
                obj.fixed_image = target.image;
                obj.u_max = size(target.image,2);
                obj.v_max = size(target.image,1);
                
                if any(source.image > 1)
                    source.image = double(source.image) / 255;
                end
                obj.moving_image = source.image;
                if any(obj.moving_pc.Intensity>1)
                    obj.moving_pc.Intensity = obj.moving_pc.Intensity./255;
                end
                obj.moving_intensity = obj.moving_pc.Intensity;
                
                % compute image intensity gradient
                [obj.imgrad.u, obj.imgrad.v] = imgradientxy(obj.fixed_image,'central');
                
                if isempty(obj.R)                   
                    obj.R = eye(3);
                    obj.T = zeros(3,1);
                end
            else
                error('Provide target and source point clouds as inputs');
            end
        end
        
        function projection(obj)   
            obj.U = obj.moving_pc_loc(:,1) * obj.fx ./ obj.moving_pc_loc(:,3) + obj.cx+1;
            obj.V = obj.moving_pc_loc(:,2) * obj.fy ./ obj.moving_pc_loc(:,3) + obj.cy+1;
            % remove out of frame projected points
            obj.invalid_points = obj.U < 0.5 | obj.U > obj.u_max | obj.V < 0.5 | obj.V > obj.v_max | isnan(obj.U);
            obj.U = round(obj.U);
            obj.U(obj.invalid_points) = [];
            obj.V = round(obj.V);
            obj.V(obj.invalid_points) = [];
            % find valid points in fixed image
            obj.valid_proj_idx = ((obj.U-1).*obj.v_max+obj.V)';
                    
            % get residual
            obj.residual = obj.fixed_image(obj.valid_proj_idx)' - obj.moving_intensity(~obj.invalid_points);
                       
        end
        
        function compute_gradient(obj)
            % Computes gradient of photometric error wrt to the rigid tf
            % parameter.
            % first project the transformed points to the fixed frame and
            % re-project them to the image.
            obj.projection();
            
            % get projected points intensity gradient
            
            obj.gradI = [];
            obj.gradI.u = obj.imgrad.u(obj.valid_proj_idx);
            obj.gradI.v = obj.imgrad.v(obj.valid_proj_idx);
                                   
            x = obj.moving_pc_loc(~obj.invalid_points,:);
            % apply Cauchy loss
            obj.alpha = 4;
            obj.J = zeros(size(x,1),6);
            for i = 1:size(x,1)
%                 try
                J_kf = [obj.fx/x(i,3), 0 , -(obj.fx*x(i,1))/x(i,3)^2;...
                    0 , obj.fy/x(i,3), -(obj.fy*x(i, 2))/x(i, 3)^2];
                Jc = J_kf * [eye(3), -obj.hat(x(i,:))];
                %obj.J(i,:) = -1/(obj.residual(i)/obj.alpha + 1) * [obj.gradI.u(i), obj.gradI.v(i)] *Jc; 
                obj.J(i,:) = [obj.gradI.u(i), obj.gradI.v(i)] *Jc; 
%                 catch
%                     keyboard;
%                 end
            end
            % correct residuals based on Cauchy loss
             %obj.residual = obj.alpha * log(1 + (obj.residual/obj.alpha));
        end
        
        function align(obj, H_GT)
            R_GT = H_GT(1:3,1:3);
            T_GT = H_GT(1:3,end);
            pcshowpair(obj.fixed_pc,obj.moving_pc);
            drawnow;
            % Aligns two RGBD point clouds
            for k = 1:obj.MAX_ITER
                % construct omega and v
                % The point clouds are fixed (x) and moving (y)
                % current transformation
                obj.tform = affine3d(([obj.R, obj.T; 0, 0, 0, 1])');
                moved = pctransform(obj.moving_pc, obj.tform);
                
                % extract point cloud information:
                obj.moving_pc_loc = double(moved.Location);
                
                % compute 3x6 Jacobian
                obj.compute_gradient();
                
                % compute step size for integrating the flow
                % obj.compute_step_size;
                % dt = obj.step;
                dt = 0.2;
                [Q_j,R_j] = qr(obj.J,0);
                twist = -dt*R_j\(Q_j'*obj.residual);
                obj.v = twist(1:3);
                obj.omega = twist(4:6);
                
                % Stop if the step size is too small
                if max(norm(obj.omega),norm(obj.v)) < obj.eps
                    break;
                end
                
                % Integrating
                
                th = norm(obj.omega); 
                homega = obj.hat(obj.omega);
                dR = eye(3) + (sin(dt * th) / th) * homega + ...
                    ((1 - cos(dt * th)) / th^2) * homega^2;
                dT = (dt * eye(3) + ...
                    (1-cos(dt * th)) / (th^2) * homega + ...
                    ((dt * th - sin(dt * th)) / th^3) * homega^2) * obj.v;
                                
                % left disturbance
                R_new = dR * obj.R;
                T_new = dR * obj.T + dT;
                
                % Update the state
                obj.R = R_new;
                obj.T = T_new;
                
                % calculate difference between currnet transformation to GT
                dR_2_GT = R_GT*obj.R';
                dT_2_GT = T_GT-dR_2_GT*obj.T;
                
                disp([k,max(norm(obj.omega),norm(obj.v)),obj.eps,obj.dist_se3(dR,dT),obj.eps_2, obj.dist_se3(dR_2_GT,dT_2_GT), sum(obj.residual.^2)]);
                % Our other break
                if obj.dist_se3(dR,dT) < obj.eps_2
                    break;
                end
                
                if mod(k,10)==0
                    pcshowpair(obj.fixed_pc,moved);
                    drawnow;
                end                
            end
            
            obj.tform = affine3d([obj.R, obj.T; 0, 0, 0, 1]');

            obj.iterations = k;
        end
    end
end

