function st = makeStation(id, description, T, P, mdot, cp, R, Tref, Pref)
% Builds one station struct: T,P plus derived h (sensible enthalpy,
% cp*T basis), s (entropy relative to the ambient reference state),
% rho (ideal gas density), mdot, and fluid identity.
st.id = id;
st.description = description;
st.T = T;                          % [K]
st.P = P;                          % [Pa]
st.h = cp * T;                     % [J/kg] sensible enthalpy (cp*T basis, not absolute)
st.s = cp*log(T/Tref) - R*log(P/Pref); % [J/kgK] relative to ambient reference state
st.rho = P / (R*T);                % [kg/m3]
st.mdot = mdot;                    % [kg/s]
if abs(cp - 1005) < abs(cp - 1100)
    st.fluid = 'air';
else
    st.fluid = 'exhaust_gas';
end
end