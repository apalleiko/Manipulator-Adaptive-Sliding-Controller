function [Y,a,qr] = collect_manipulator_terms(eqn,q_ddot,q_dot,q,vars)
% Collect terms of the manipulator equation into corresponding matrices
% H(q)*q_ddot_r + C(q,q_dot)*q_dot_r + g(q) = Y(q,q_dot,q_dot_r,q_ddot_r)*a
q_ddot = formula(q_ddot);
q_dot = formula(q_dot);
q = formula(q);

n = length(formula(q));
m = length(formula(eqn));

% First arrange into manipulator equation matrices
G = eqn;

M = symmatrix('M',[n n]);
q_ddot_r = sym('q%d_ddot_r',[n,1]);
for i=1:m
    for j=1:n
        out = children(collect(eqn(i),q_ddot(j)),1);
        G(i) = G(i) - out;
        M(i,j) = simplify(out/q_ddot(j));
    end
end

syms ph

C = symmatrix('C',[n n]);
q_dot_r = sym('q%d_dot_r',[n,1]);
for i=1:m
    for j=1:n
        out = children(collect(eqn(i),q_dot(j)),1);
        out2 = subs(out,q_dot(j),ph);
        if has(out2, ph)
            G(i) = G(i) - out;
            C(i,j) = simplify(out/q_dot(j));
        else
            C(i,j) = 0;
        end
    end
end
G = symmatrix(simplify(G));

% Now find the coeffs of the polynomial combinations of unknown vars
% Later only do this for a desired unknown, like EE intertia?
Ya = symmatrix2sym(M*q_ddot_r + C*q_dot_r + G);

Y_temp = {};
a_temp = {};
for i=1:n
    % Get variable coefficients for each equation in terms of known
    % quantities
    [Yi,ai] = coeffs(Ya(i),vars);
    num = length(ai);
    for k = 1:num
        % Check if specific a exists already
        f = @(x) isAlways(x==ai(k),"Unknown","false");
        cb = cellfun(f,a_temp);
        if isempty(cb) | ~any(cb)
            a_temp{end+1} = ai(k);
            Y_temp{i,end+1} = Yi(k);
        else
            if sum(cb)>1
                error("MORE THAN 1 EQUALITY FOUND")
            end
            idx = find(cb);
            Y_temp{i,idx} = Yi(k);
        end
    end
end

% Now can try and combine terms with the same multiplier in Y
Y_temp2 = {};
a_temp2 = {};
blacklist = [];
for j=1:size(Y_temp,2)
    % Check that this index isn't blacklisted (combined) yet
    if ismember(j,blacklist)
        continue
    end

    % Check that all nonzero terms in the column are equal
    Yj = Y_temp(:,j);
    Y_temp2(:,end+1) = Yj; % Always true for a nonconsolidated column
    aj = a_temp{j};
    nz = ~cellfun(@isempty, Yj); % Get indices of nonzero terms
    nz_terms = Yj(nz); % Get nonzero terms
    eq = all(cellfun(@(x) isAlways(x==nz_terms{1},"Unknown","false"),nz_terms));
    if ~eq
        a_temp2{end+1} = aj; % Don't consider nonequal (nonnull) columns
        blacklist = [blacklist, j];
        continue
    end
    
    % Divide all other entries by the term
    alt = cellfun(@(x) x/nz_terms{1},Y_temp,UniformOutput=false);
    % Check for other columns with the same zero terms and a constant
    % multiplier
    for k=1:size(Y_temp,2)
        % Don't reconsider consolidated or established nonequal columns
        if ismember(k,blacklist) || j==k
            continue
        end
        alt_k = alt(:,k);
        nzk = ~cellfun(@isempty, alt_k); % Get indices of nonzero terms
        nzk_terms = alt_k(nzk); 
        % Check if nonzero terms are equal
        eqm = all(cellfun(@(x) isAlways(x==nzk_terms{1},"Unknown","false"),nzk_terms));
        % Check if nonzero terms are constant
        cons = all(cellfun(@(x) isempty(symvar(simplify(x))),nzk_terms));
        if nzk==nz & eqm & cons
            % If all conditions met, can consolidate a term for this column
            mult = nzk_terms{1}; % This is the multiplier
            blacklist = [blacklist, k]; % Add column to blacklist
            aj = aj + mult*a_temp{k}; % Add to existing aj term
        end
    end
    a_temp2{end+1} = simplify(aj);
end

Y = symmatrix('Y',size(Y_temp2));
a = symmatrix('a',fliplr(size(a_temp2)));
for i=1:size(Y_temp2,2)
    a(i) = a_temp2{i};
    for j=1:size(Y_temp2,1)
        if isempty(Y_temp2{j,i})
            Y(j,i) = 0;
        else
            Y(j,i) = Y_temp2{j,i};
        end
    end
end

qr = [q_dot_r; q_ddot_r];

if ~all(isAlways(simplify(formula(symmatrix2sym((Y*a)))==Ya)))
    error("Computed Ya not equal to Y*a")
end

Y = -Y;

end

