function mCADplot(drone,option,axis)
% Plot ArDrone 3D CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T
    

drone.pPos.Xc = drone.pPos.X;


if nargin < 2
    drone.pCAD.flagFrame = 0;
    option = 'indoor';       % Rotacionar pás durante a simulação
end


if drone.pCAD.flagCreated == 0
    drone.pPos.psiHel = 0;
    
    drone.pCAD.flagFrame = 1;

    mCADmake(drone)
    mCADplot(drone,option)
    
else
    
    
    if nargin < 3
        drone.pCAD.flagFrame = 0; % Colocar isso do lado de fora do código!
    else
        drone.pCAD.flagFrame = axis;
    end
    
    switch option
        
        case 'outdoor'
            model = 1;
            drone.pCAD.i3D{1}.Visible = 'on';
            drone.pCAD.i3D{2}.Visible = 'off';

            for idx = 3:6
                drone.pCAD.i3D{idx}.Visible = 'on';
                drone.pCAD.i3D{idx}.FaceAlpha = 1;
            end
            
            drone.pCAD.OriginalAxis{2}(2,2) = 0.30;
            
        case 'indoor'
            model = 2;
            drone.pCAD.i3D{1}.Visible = 'off';

            for idx = 3:6
                drone.pCAD.i3D{idx}.Visible = 'on';
                drone.pCAD.i3D{idx}.FaceAlpha = 1.0;
            end
            
            drone.pCAD.OriginalAxis{2}(2,2) = 0.15;
    end
    
    drone.pCAD.i3D{model}.FaceAlpha = 1;
    drone.pCAD.i3D{model}.Visible = 'on';
    
    
    % Update robot pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(drone.pPos.Xc(4)) -sin(drone.pPos.Xc(4)); 0 sin(drone.pPos.Xc(4)) cos(drone.pPos.Xc(4))];
    RotY = [cos(drone.pPos.Xc(5)) 0 sin(drone.pPos.Xc(5)); 0 1 0; -sin(drone.pPos.Xc(5)) 0 cos(drone.pPos.Xc(5))];
    RotZ = [cos(drone.pPos.Xc(6)) -sin(drone.pPos.Xc(6)) 0; sin(drone.pPos.Xc(6)) cos(drone.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot drone.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[drone.pCAD.obj{1}.v; ones(1,size(drone.pCAD.obj{1}.v,2))];
    drone.pCAD.i3D{1}.Vertices = vertices(1:3,:)';

    
    if drone.pCAD.flagFrame 
        
        drone.pCAD.i3D{7}.Visible = 'on';
        drone.pCAD.i3D{8}.Visible = 'on';
        drone.pCAD.i3D{9}.Visible = 'on';
        
        xAxis = [drone.pCAD.OriginalAxis{1}(1,:); drone.pCAD.OriginalAxis{2}(1,:); drone.pCAD.OriginalAxis{3}(1,:)];
        vertBase = H*[xAxis; ones(1,size(xAxis,2))];
        drone.pCAD.i3D{7}.XData = vertBase(1,:)';
        drone.pCAD.i3D{7}.YData = vertBase(2,:)';
        drone.pCAD.i3D{7}.ZData = vertBase(3,:)';
        
        yAxis = [drone.pCAD.OriginalAxis{1}(2,:); drone.pCAD.OriginalAxis{2}(2,:); drone.pCAD.OriginalAxis{3}(2,:)];
        vertBase = H*[yAxis; ones(1,size(yAxis,2))];
        drone.pCAD.i3D{8}.XData = vertBase(1,:)';
        drone.pCAD.i3D{8}.YData = vertBase(2,:)';
        drone.pCAD.i3D{8}.ZData = vertBase(3,:)';
        
        zAxis = [drone.pCAD.OriginalAxis{1}(3,:); drone.pCAD.OriginalAxis{2}(3,:); drone.pCAD.OriginalAxis{3}(3,:)]; 
        vertBase = H*[zAxis; ones(1,size(zAxis,2))];
        drone.pCAD.i3D{9}.XData = vertBase(1,:)';
        drone.pCAD.i3D{9}.YData = vertBase(2,:)';
        drone.pCAD.i3D{9}.ZData = vertBase(3,:)';
        
    else
        
        drone.pCAD.i3D{7}.Visible = 'off';
        drone.pCAD.i3D{8}.Visible = 'off';
        drone.pCAD.i3D{9}.Visible = 'off';
    end

   

    % ----- Drone de pás rotativas:
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(drone.pPos.Xc(4)) -sin(drone.pPos.Xc(4)); 0 sin(drone.pPos.Xc(4)) cos(drone.pPos.Xc(4))];
    RotY = [cos(drone.pPos.Xc(5)) 0 sin(drone.pPos.Xc(5)); 0 1 0; -sin(drone.pPos.Xc(5)) 0 cos(drone.pPos.Xc(5))];
    RotZ = [cos(drone.pPos.Xc(6)) -sin(drone.pPos.Xc(6)) 0; sin(drone.pPos.Xc(6)) cos(drone.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot drone.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[drone.pCAD.obj{2}.v; ones(1,size(drone.pCAD.obj{2}.v,2))];
    drone.pCAD.i3D{2}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Frente Dir.)
    RotX = [1 0 0; 0 cos(drone.pPos.Xc(4)) -sin(drone.pPos.Xc(4)); 0 sin(drone.pPos.Xc(4)) cos(drone.pPos.Xc(4))];
    RotY = [cos(drone.pPos.Xc(5)) 0 sin(drone.pPos.Xc(5)); 0 1 0; -sin(drone.pPos.Xc(5)) 0 cos(drone.pPos.Xc(5))];
    RotZ = [cos(drone.pPos.Xc(6)) -sin(drone.pPos.Xc(6)) 0; sin(drone.pPos.Xc(6)) cos(drone.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [0.126; 0.127; 0]; 0 0 0 1];
    
    
    Rot_J1 = [ cos(drone.pPos.psiHel) sin(drone.pPos.psiHel) 0;
              -sin(drone.pPos.psiHel) cos(drone.pPos.psiHel) 0;
                        0                    0               1];
            
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{3}.v; ones(1,size(drone.pCAD.obj{3}.v,2))];
    drone.pCAD.i3D{3}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Tras Esq.)
    RotX = [1 0 0; 0 cos(drone.pPos.Xc(4)) -sin(drone.pPos.Xc(4)); 0 sin(drone.pPos.Xc(4)) cos(drone.pPos.Xc(4))];
    RotY = [cos(drone.pPos.Xc(5)) 0 sin(drone.pPos.Xc(5)); 0 1 0; -sin(drone.pPos.Xc(5)) 0 cos(drone.pPos.Xc(5))];
    RotZ = [cos(drone.pPos.Xc(6)) -sin(drone.pPos.Xc(6)) 0; sin(drone.pPos.Xc(6)) cos(drone.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [-0.126; -0.127; 0]; 0 0 0 1];

                    
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{3}.v; ones(1,size(drone.pCAD.obj{3}.v,2))];
    drone.pCAD.i3D{4}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Frente Esq.)
    RotX = [1 0 0; 0 cos(drone.pPos.Xc(4)) -sin(drone.pPos.Xc(4)); 0 sin(drone.pPos.Xc(4)) cos(drone.pPos.Xc(4))];
    RotY = [cos(drone.pPos.Xc(5)) 0 sin(drone.pPos.Xc(5)); 0 1 0; -sin(drone.pPos.Xc(5)) 0 cos(drone.pPos.Xc(5))];
    RotZ = [cos(drone.pPos.Xc(6)) -sin(drone.pPos.Xc(6)) 0; sin(drone.pPos.Xc(6)) cos(drone.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [0.126; -0.127; 0]; 0 0 0 1];
    
    Rot_J1 = [ cos(-(drone.pPos.psiHel + pi/2)) sin(-(drone.pPos.psiHel + pi/2)) 0;
              -sin(-(drone.pPos.psiHel + pi/2)) cos(-(drone.pPos.psiHel + pi/2)) 0;
                              0                              0               1];
                          
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{3}.v; ones(1,size(drone.pCAD.obj{3}.v,2))];
    drone.pCAD.i3D{5}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Tras Dir.)
    RotX = [1 0 0; 0 cos(drone.pPos.Xc(4)) -sin(drone.pPos.Xc(4)); 0 sin(drone.pPos.Xc(4)) cos(drone.pPos.Xc(4))];
    RotY = [cos(drone.pPos.Xc(5)) 0 sin(drone.pPos.Xc(5)); 0 1 0; -sin(drone.pPos.Xc(5)) 0 cos(drone.pPos.Xc(5))];
    RotZ = [cos(drone.pPos.Xc(6)) -sin(drone.pPos.Xc(6)) 0; sin(drone.pPos.Xc(6)) cos(drone.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [-0.126; 0.127; 0]; 0 0 0 1];
                        
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{3}.v; ones(1,size(drone.pCAD.obj{3}.v,2))];
    drone.pCAD.i3D{6}.Vertices = vertices(1:3,:)';
   

end


drone.pPos.psiHel = drone.pPos.psiHel + (1 + (1.05-1)*rand(1))*pi/6;


end

% =========================================================================
function mCADmake(uav)

for i = 1:length(uav.pCAD.obj)
    
    hold on
    uav.pCAD.i3D{i} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    % size(fvcd3)
    
    uav.pCAD.i3D{i}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{i}.FaceColor = 'flat';
    uav.pCAD.i3D{i}.EdgeColor = 'none';
    uav.pCAD.i3D{i}.FaceAlpha = 0.0;
    uav.pCAD.i3D{i}.Visible = 'off';
    % light;

end

i = 3;
for iP = 4:6
    hold on
    uav.pCAD.i3D{iP} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    
    uav.pCAD.i3D{iP}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{iP}.FaceColor = 'flat';
    uav.pCAD.i3D{iP}.EdgeColor = 'none';
    uav.pCAD.i3D{iP}.FaceAlpha = 0.0;
    uav.pCAD.i3D{iP}.Visible = 'off';
end
    
uav.pCAD.flagCreated = 1;


if uav.pCAD.flagFrame
    hold on
    uav.pCAD.i3D{length(uav.pCAD.i3D)+1} = plot3([0 .30],[0 0],  [0 0], '-r','LineWidth',1,'Marker','o','MarkerSize',1,'MarkerEdgeColor','r','MarkerFaceColor','k','MarkerIndices',2);
    uav.pCAD.i3D{length(uav.pCAD.i3D)+1} = plot3([0 0],  [0 .15],[0 0], '-g','LineWidth',1,'Marker','o','MarkerSize',1,'MarkerEdgeColor','g','MarkerFaceColor','k','MarkerIndices',2);
    uav.pCAD.i3D{length(uav.pCAD.i3D)+1} = plot3([0 0],  [0 0],  [0 .15],'-b','LineWidth',1,'Marker','o','MarkerSize',1,'MarkerEdgeColor','b','MarkerFaceColor','k','MarkerIndices',2);
    hold off

    uav.pCAD.OriginalAxis{1} = [0 .30;
                                0   0;
                                0   0];
    uav.pCAD.OriginalAxis{2} = [0   0;
                                0 .15;
                                0   0];
    uav.pCAD.OriginalAxis{3} = [0   0;
                                0   0;
                                0 .15];

    uav.pCAD.i3D{7}.Visible = 'off';
    uav.pCAD.i3D{8}.Visible = 'off';
    uav.pCAD.i3D{9}.Visible = 'off';                            
   
end

drawnow limitrate nocallbacks


end