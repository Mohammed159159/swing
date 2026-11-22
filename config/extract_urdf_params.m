function cad = extract_urdf_params(urdf_path)
%EXTRACT_URDF_PARAMS Extract Furuta mechanical scalars (I0, L0, m1, l1, J1) from URDF.
%   cad = EXTRACT_URDF_PARAMS() parses digital-twin/simscape/robot.urdf relative
%   to the repository root.
%   cad = EXTRACT_URDF_PARAMS(urdf_path) parses the specified URDF file, rotates
%   inertial tensors by inertial <origin rpy>, applies the parallel-axis theorem
%   around the rotary arm joint axis for I0, and projects onto the pendulum joint
%   axis for l1 and J1.

    if nargin < 1 || isempty(urdf_path)
        repoRoot  = fileparts(fileparts(mfilename('fullpath')));
        urdf_path = fullfile(repoRoot, 'digital-twin', 'simscape', 'robot.urdf');
    end
    if ~isfile(urdf_path)
        error('Furuta:FileNotFound', 'URDF file not found: %s', urdf_path);
    end

    doc    = xmlread(urdf_path);
    links  = parseLinks(doc);
    joints = parseJoints(doc);

    [armJoint, pendJoint] = resolveKinematicChain(links, joints);

    armLink  = links(armJoint.child);
    pendLink = links(pendJoint.child);

    a1 = unitVec(armJoint.axis);
    a2 = unitVec(pendJoint.axis);

    % L0: radial distance from base rotation axis (a1) to pendulum pivot origin
    p_j2_perp = pendJoint.xyz - dot(pendJoint.xyz, a1) * a1;
    L0        = norm(p_j2_perp);

    % I0: arm inertia about joint 1 axis via parallel-axis theorem
    r_arm_perp = armLink.com_xyz - dot(armLink.com_xyz, a1) * a1;
    I_arm_rot  = rotateInertia(armLink.I_com, armLink.com_rpy);
    I0         = a1.' * I_arm_rot * a1 + armLink.mass * sum(r_arm_perp.^2);

    % m1, l1, J1: pendulum mass, perpendicular COM offset from joint 2 axis, and COM inertia about a2
    m1          = pendLink.mass;
    r_pend_perp = pendLink.com_xyz - dot(pendLink.com_xyz, a2) * a2;
    l1          = norm(r_pend_perp);
    I_pend_rot  = rotateInertia(pendLink.I_com, pendLink.com_rpy);
    J1          = a2.' * I_pend_rot * a2;

    if L0 < 0.02
        warning('Furuta:SmallArmLength', ...
            'Extracted L0 = %.6f m (< 0.02 m). Check URDF joint "%s" origin offset.', ...
            L0, pendJoint.name);
    end

    cad = struct( ...
        'urdf_path', char(urdf_path), ...
        'arm_link',  armJoint.child, ...
        'pend_link', pendJoint.child, ...
        'm0',        armLink.mass, ...
        'I0',        I0, ...
        'L0',        L0, ...
        'm1',        m1, ...
        'l1',        l1, ...
        'J1',        J1);

    fprintf('--- Extracted URDF Parameters (%s) ---\n', urdf_path);
    fprintf('  params.I0 = %.8g;  %% [kg*m^2] (arm mass m0 = %.6f kg)\n', I0, armLink.mass);
    fprintf('  params.L0 = %.8g;  %% [m]\n', L0);
    fprintf('  params.m1 = %.8g;  %% [kg]\n', m1);
    fprintf('  params.l1 = %.8g;  %% [m]\n', l1);
    fprintf('  params.J1 = %.8g;  %% [kg*m^2]\n', J1);
end

function links = parseLinks(doc)
    links    = containers.Map('KeyType', 'char', 'ValueType', 'any');
    linkList = doc.getElementsByTagName('link');
    for i = 0:linkList.getLength() - 1
        node = linkList.item(i);
        name = char(node.getAttribute('name'));
        info = struct('name', name, 'mass', 0, ...
            'com_xyz', [0; 0; 0], 'com_rpy', [0; 0; 0], 'I_com', zeros(3));

        inertNode = firstDirectChild(node, 'inertial');
        if ~isempty(inertNode)
            origNode = firstDirectChild(inertNode, 'origin');
            if ~isempty(origNode)
                info.com_xyz = parseVec3(origNode.getAttribute('xyz'), [0; 0; 0]);
                info.com_rpy = parseVec3(origNode.getAttribute('rpy'), [0; 0; 0]);
            end
            massNode = firstDirectChild(inertNode, 'mass');
            if ~isempty(massNode)
                info.mass = str2double(char(massNode.getAttribute('value')));
            end
            ineNode = firstDirectChild(inertNode, 'inertia');
            if ~isempty(ineNode)
                ixx = str2double(char(ineNode.getAttribute('ixx')));
                ixy = str2double(char(ineNode.getAttribute('ixy')));
                ixz = str2double(char(ineNode.getAttribute('ixz')));
                iyy = str2double(char(ineNode.getAttribute('iyy')));
                iyz = str2double(char(ineNode.getAttribute('iyz')));
                izz = str2double(char(ineNode.getAttribute('izz')));
                info.I_com = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
            end
        end
        links(name) = info;
    end
end

function joints = parseJoints(doc)
    joints    = struct('name', {}, 'type', {}, 'parent', {}, 'child', {}, ...
        'xyz', {}, 'rpy', {}, 'axis', {});
    jointList = doc.getElementsByTagName('joint');
    for i = 0:jointList.getLength() - 1
        node = jointList.item(i);
        j.name   = char(node.getAttribute('name'));
        j.type   = char(node.getAttribute('type'));
        j.parent = char(firstDirectChild(node, 'parent').getAttribute('link'));
        j.child  = char(firstDirectChild(node, 'child').getAttribute('link'));

        origNode = firstDirectChild(node, 'origin');
        if ~isempty(origNode)
            j.xyz = parseVec3(origNode.getAttribute('xyz'), [0; 0; 0]);
            j.rpy = parseVec3(origNode.getAttribute('rpy'), [0; 0; 0]);
        else
            j.xyz = [0; 0; 0];
            j.rpy = [0; 0; 0];
        end

        axisNode = firstDirectChild(node, 'axis');
        if ~isempty(axisNode)
            j.axis = parseVec3(axisNode.getAttribute('xyz'), [1; 0; 0]);
        else
            j.axis = [1; 0; 0];
        end
        joints(end+1) = j; %#ok<AGROW>
    end
end

function [armJoint, pendJoint] = resolveKinematicChain(links, joints)
    allLinks  = links.keys;
    children  = {joints.child};
    rootLinks = setdiff(allLinks, children);
    if isempty(rootLinks)
        error('Furuta:InvalidURDF', 'Could not determine root link in URDF.');
    end

    rootName = rootLinks{1};
    j1Idx    = find(strcmp({joints.parent}, rootName), 1);
    if isempty(j1Idx)
        error('Furuta:InvalidURDF', 'No joint attached to root link "%s".', rootName);
    end
    armJoint = joints(j1Idx);

    j2Idx = find(strcmp({joints.parent}, armJoint.child), 1);
    if isempty(j2Idx)
        error('Furuta:InvalidURDF', 'No pendulum joint attached to arm link "%s".', armJoint.child);
    end
    pendJoint = joints(j2Idx);
end

function child = firstDirectChild(parentNode, tagName)
    child = [];
    kids  = parentNode.getChildNodes();
    for i = 0:kids.getLength() - 1
        item = kids.item(i);
        if item.getNodeType() == 1 && strcmp(char(item.getNodeName()), tagName)
            child = item;
            return;
        end
    end
end

function v = parseVec3(attrStr, defaultVec)
    s = strtrim(char(attrStr));
    if isempty(s)
        v = defaultVec;
        return;
    end
    vals = sscanf(s, '%f');
    if numel(vals) ~= 3
        error('Furuta:InvalidURDF', 'Expected 3-element vector, got "%s".', s);
    end
    v = vals(:);
end

function u = unitVec(v)
    n = norm(v);
    if n <= 0
        error('Furuta:InvalidURDF', 'Zero-norm joint axis encountered.');
    end
    u = v(:) / n;
end

function I_rot = rotateInertia(I_com, rpy)
    if all(rpy == 0)
        I_rot = I_com;
        return;
    end
    r = rpy(1); p = rpy(2); y = rpy(3);
    Rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
    Ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
    Rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
    R  = Rz * Ry * Rx;
    I_rot = R * I_com * R.';
end
