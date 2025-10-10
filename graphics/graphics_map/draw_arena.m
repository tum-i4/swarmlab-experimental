function fig_handle = draw_arena(fig_handle, map)
% draw_arena - plot arena walls as a transparent gray box
  
  vertices = [];
  faces = [];
  patch_colors = [];

  e = map.arena_east;
  n = map.arena_north;
  z = map.arena_down;

  vertices_temp = [...
        e(1), n(1), -z(1);...
        e(1), n(2), -z(1);...
        e(2), n(2), -z(1);...
        e(2), n(1), -z(1);...
        e(1), n(1), -z(2);...
        e(1), n(2), -z(2);...
        e(2), n(2), -z(2);...
        e(2), n(1), -z(2);...
        ];    
  vertices = [vertices; vertices_temp];
  faces_temp = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   
  faces = [faces; faces_temp];
  my_light_gray = [0.9, 0.9, 0.9];

  patch_colors_temp = [...
    my_light_gray;...      % North
    my_light_gray;...      % East
    my_light_gray;...      % South
    my_light_gray;...      % West
    my_light_gray;...      % Top
    ];


  % % For creating trajectory figure 1 for paper
  % % Comment out the above code to replace it with the below.
  % faces_temp = [...
  %       1, 4, 8, 5;... % North Side
  %       1, 2, 6, 5;... % East Side
  %       2, 3, 7, 6;... % South Side
  %       % 3, 4, 8, 7;... % West Side
  %       5, 6, 7, 8;... % Top
  %       1, 2, 3, 4;... % Bottom
  %       ];   
  % faces = [faces; faces_temp];
  % my_light_gray = [86, 180, 233]./255;
  % patch_colors_temp = [...
  %   my_light_gray;...      % North
  %   my_light_gray;...      % East
  %   my_light_gray;...      % South
  %   % my_light_gray;...      % West
  %   my_light_gray;...      % Top
  %   my_light_gray;...      % Bottom
  %   ];

  % % For creating trajectory figure 2 for paper
  % % Comment out the above code to replace it with the below.
  % faces_temp = [...
  %       1, 4, 8, 5;... % North Side
  %       1, 2, 6, 5;... % East Side
  %       2, 3, 7, 6;... % South Side
  %       3, 4, 8, 7;... % West Side
  %       5, 6, 7, 8;... % Top
  %       % 1, 2, 3, 4;... % Bottom
  %       ];   
  % faces = [faces; faces_temp];
  % my_light_gray = [86, 180, 233]./255;
  % patch_colors_temp = [...
  %   my_light_gray;...      % North
  %   my_light_gray;...      % East
  %   my_light_gray;...      % South
  %   my_light_gray;...      % West
  %   my_light_gray;...      % Top
  %   % my_light_gray;...      % Bottom
  %   ];


  patch_colors = [patch_colors;patch_colors_temp];
  
  patch('Vertices', vertices, 'Faces', faces,...
      'FaceVertexCData',patch_colors,...
      'FaceAlpha',0.3,...
      'FaceColor','flat');
  % axis_len = map.width;
  % margin = axis_len/5;
  % axes_lim = [-margin, axis_len + margin, ...
  %     -margin, axis_len+margin, ...
  %     0, 1.2 * map.max_height];
  % axis(axes_lim);
  % % axis equal;
  % view(32, 47);
  hold on;

end
