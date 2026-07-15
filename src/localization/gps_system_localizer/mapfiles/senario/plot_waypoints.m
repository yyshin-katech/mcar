function plot_waypoints()
% PLOT_WAYPOINTS  mat 폴더의 east/north 좌표 시각화 (하네스)
%
%   현재 .m 파일이 위치한 폴더의 link_*.mat 파일들을 읽어,
%     - east 를 x축, north 를 y축 으로
%     - 각 링크의 모든 포인트를 (선이 아닌) 점으로 표시
%     - OSM 배경 지도 위에 겹쳐 그림
%     - 점 클릭 시 우측 패널에 해당 링크의 변수 정보 표시
%
%   배경 이미지(background.png, background_bbox.mat)가 없으면 먼저:
%     python fetch_map_bg.py mat/<folder>
%
%   사용:
%     >> cd mat/senario_260514c
%     >> plot_waypoints

    folder = fileparts(mfilename('fullpath'));
    fprintf('[plot_waypoints] 폴더: %s\n', folder);

    % ── 1) 배경 이미지 + bbox ─────────────────────────────────────
    bg_png = fullfile(folder, 'background.png');
    bg_mat = fullfile(folder, 'background_bbox.mat');
    has_bg = (exist(bg_png, 'file') == 2) && (exist(bg_mat, 'file') == 2);
    if ~has_bg
        warning('plot_waypoints:noBackground', ...
            ['배경 이미지 없음. 먼저 다음 명령을 실행하세요:\n' ...
             '    python fetch_map_bg.py %s'], folder);
    end

    % ── 2) mat 파일 로드 ─────────────────────────────────────────
    files = dir(fullfile(folder, 'link_*.mat'));
    n_files = numel(files);
    fprintf('[plot_waypoints] mat 파일: %d개\n', n_files);
    if n_files == 0
        error('link_*.mat 파일이 없습니다.');
    end

    links = repmat(struct('fid', 0, 'east', [], 'north', [], ...
                          'link_id_string', '', 'data', struct()), ...
                   n_files, 1);
    for i = 1:n_files
        fname = files(i).name;
        d = load(fullfile(folder, fname));
        tok = regexp(fname, 'link_(\d+)\.mat', 'tokens', 'once');
        links(i).fid = str2double(tok{1});
        links(i).east  = double(d.east(:));
        links(i).north = double(d.north(:));
        lid = '';
        if isfield(d, 'LINK_ID_string')
            v = d.LINK_ID_string;
            if iscell(v), v = v{1}; end
            if ischar(v) || isstring(v), lid = strtrim(char(v)); end
        end
        links(i).link_id_string = lid;
        links(i).data = d;
    end

    % ── 3) Figure / Axes ─────────────────────────────────────────
    [~, fname_only, ~] = fileparts(folder);
    fig = figure('Name', sprintf('mat 좌표 뷰어 — %s', fname_only), ...
                 'NumberTitle', 'off', ...
                 'Position', [60 60 1500 920], ...
                 'Color', 'w');
    ax = axes('Parent', fig, 'Position', [0.05 0.07 0.70 0.88]);
    hold(ax, 'on');

    % 배경 이미지: 첫 row = 북쪽 이므로 YData=[north_max north_min] 으로 매핑
    if has_bg
        bbox = load(bg_mat);
        img = imread(bg_png);
        image('Parent', ax, 'CData', img, ...
              'XData', [bbox.east_min  bbox.east_max], ...
              'YData', [bbox.north_max bbox.north_min], ...
              'HitTest', 'off', 'PickableParts', 'none');
    end
    set(ax, 'YDir', 'normal');

    % ── 4) 점 (링크별 plot, ButtonDownFcn) ───────────────────────
    cmap = lines(7);
    handles = gobjects(n_files, 1);
    for i = 1:n_files
        L = links(i);
        h = plot(ax, L.east, L.north, '.', ...
                 'Color', cmap(mod(i-1, 7) + 1, :), ...
                 'MarkerSize', 10, ...
                 'LineStyle', 'none');
        h.UserData = i;
        h.ButtonDownFcn = @onClick;
        handles(i) = h;
    end

    axis(ax, 'equal');
    xlabel(ax, 'East (m, EPSG:5179)');
    ylabel(ax, 'North (m, EPSG:5179)');
    title(ax, sprintf('%s — %d 링크 (점 클릭 시 정보)', ...
                       strrep(fname_only, '_', '\_'), n_files), ...
          'Interpreter', 'tex');
    grid(ax, 'on');
    set(ax, 'Layer', 'top', 'Box', 'on');

    % 데이터 bbox 에 맞춰 zoom
    all_e = vertcat(links.east);
    all_n = vertcat(links.north);
    if ~isempty(all_e)
        pad = 50;
        xlim(ax, [min(all_e) - pad, max(all_e) + pad]);
        ylim(ax, [min(all_n) - pad, max(all_n) + pad]);
    end

    % ── 5) 정보 패널 ─────────────────────────────────────────────
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.77 0.955 0.22 0.030], ...
        'String', '선택 링크 정보', 'FontWeight', 'bold', 'FontSize', 11, ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'left');

    info = uicontrol('Parent', fig, 'Style', 'listbox', ...
        'Units', 'normalized', 'Position', [0.77 0.07 0.22 0.88], ...
        'FontName', 'Consolas', 'FontSize', 11, ...
        'BackgroundColor', [0.98 0.98 0.96], ...
        'String', {'점을 클릭하면 해당 링크 정보가 표시됩니다.', ...
                   '', ...
                   sprintf('현재 로드: %d개 링크', n_files)});

    % ── 6) appdata 저장 + 콜백 등록 ──────────────────────────────
    setappdata(fig, 'info', info);
    setappdata(fig, 'links', links);
    setappdata(fig, 'handles', handles);
    setappdata(fig, 'ax', ax);
    setappdata(fig, 'sel_handle', []);
    setappdata(fig, 'sel_idx', 0);

    fprintf('[plot_waypoints] 준비 완료. 점을 클릭하세요.\n');
end


% ─────────────────────────────────────────────────────────────────
function onClick(src, ~)
    fig    = ancestor(src, 'figure');
    info   = getappdata(fig, 'info');
    links  = getappdata(fig, 'links');
    prev   = getappdata(fig, 'sel_handle');

    % 이전 하이라이트 복원
    if ~isempty(prev) && isgraphics(prev) && prev ~= src
        set(prev, 'MarkerSize', 10, 'Marker', '.', 'LineWidth', 0.5);
    end
    set(src, 'MarkerSize', 16, 'Marker', 'o', 'LineWidth', 1.8);

    idx = src.UserData;
    setappdata(fig, 'sel_handle', src);
    setappdata(fig, 'sel_idx', idx);

    L = links(idx);
    d = L.data;

    lines_ = {};
    lines_{end+1} = sprintf('=== link_%d ===', L.fid); %#ok<*AGROW>
    lines_{end+1} = sprintf('LINK_ID_string : %s', L.link_id_string);
    lines_{end+1} = sprintf('point_count    : %d', numel(L.east));
    lines_{end+1} = '';

    % 스칼라 필드들 (있는 것만 표시)
    flds = {'LINK_ID', 'NEXT_LINK_ID', 'RIGHT_LINK_ID', 'LEFT_LINK_ID', ...
            'Speed_Limit', ...
            'is_stop_line', 'guard_zone', ...
            'look_at_signalGroupID', 'look_at_IntersectionID', ...
            'have_to_LangeChange_right', 'have_to_LangeChange_left', ...
            'right_LaneChange_avail',    'left_LaneChange_avail'};
    lines_{end+1} = '-- 스칼라 필드 --';
    for k = 1:numel(flds)
        f = flds{k};
        if isfield(d, f)
            v = d.(f);
            if numel(v) == 1
                lines_{end+1} = sprintf('  %-26s: %d', f, double(v));
            end
        end
    end

    % 좌표
    lines_{end+1} = '';
    lines_{end+1} = '-- 좌표 (EPSG:5179) --';
    lines_{end+1} = sprintf('  start E,N : %.2f, %.2f', ...
                            L.east(1),   L.north(1));
    lines_{end+1} = sprintf('  end   E,N : %.2f, %.2f', ...
                            L.east(end), L.north(end));
    if isfield(d, 'station')
        s = double(d.station(:));
        if ~isempty(s)
            lines_{end+1} = sprintf('  length    : %.2f m', s(end));
            if numel(s) > 1
                lines_{end+1} = sprintf('  avg step  : %.2f m', ...
                                        s(end) / (numel(s) - 1));
            end
        end
    end

    % 클릭 위치
    try
        cp = get(get(src, 'Parent'), 'CurrentPoint');
        lines_{end+1} = '';
        lines_{end+1} = sprintf('  click E,N : %.2f, %.2f', ...
                                cp(1, 1), cp(1, 2));
    catch
    end

    set(info, 'String', lines_, 'Value', 1);
    fprintf('  선택: link_%d (%s)\n', L.fid, L.link_id_string);
end
