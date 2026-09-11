function P_shed = loadShed(n, n_ref, m_f, m_f_max)
%LOADSHED  Proportional load back-off for the max-fuel (underspeed) limiter.
%   Continuous, chatter-free power OFFSET to SUBTRACT from the load command.
%   Returns 0 in normal operation. When the shaft droops (n < n_ref) AND fuel is
%   at its dynamic ceiling (m_f -> m_f_max), it returns a positive power offset
%   PROPORTIONAL to the speed deficit, so the applied load is reduced just enough
%   to let the shaft recover. Being proportional (not rate/integral) it adds no
%   extra phase lag, so the loop keeps its phase margin and does not limit-cycle.
%
%   Use:  target = raw_request - P_shed;   P_el_cmd = RateLimiter(target)
%   Subtract BEFORE the rate limiter so it tracks the shed-adjusted target and
%   cannot wind up. Put a Unit Delay (Ts = fixed step, IC = 0) on P_shed to break
%   the algebraic loop, since n_ref/m_f depend on P_el_cmd.
%
%   Inputs (scalars):  n, n_ref [rpm];  m_f, m_f_max [kg/s]
%   Output:            P_shed  [W]  (>= 0, amount to subtract from the load)

    % ---- tuning (starting points) -------------------------------------------
    K           = 30;      % droop gain: W of back-off per rpm of deficit [W/rpm]
    DZ          = 100;     % speed-deficit deadband                       [rpm]
    P_SHED_MAX  = 15e3;    % max load back-off                            [W]
    FUEL_BAND   = 0.04;    % fuel fade-in band, fraction of m_f_max

    % ---- speed deficit beyond the deadband ----------------------------------
    e      = n_ref - n;              % > 0 when the shaft is lagging
    shed_e = max(0, e - DZ);         % ignore the normal transient lag

    % ---- smooth fuel-saturation factor s in [0,1] ---------------------------
    band = FUEL_BAND * m_f_max;                    % [kg/s]
    if band <= 0
        s = double(m_f >= m_f_max);
    else
        s = (m_f - (m_f_max - band)) / band;       % 0 with margin, 1 at the ceiling
        s = min(max(s, 0), 1);
    end

    % ---- proportional droop -------------------------------------------------
    P_shed = K * s * shed_e;
    P_shed = min(max(P_shed, 0), P_SHED_MAX);      % clamp to [0, P_SHED_MAX]
end
