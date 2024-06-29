classdef wlar_model
    properties
        totalmass = 80;
        % torque limits for each leg 
        % HR = Hip roll
        % HP = Hip pitch
        % K = Knee 
        % W = wheel

        LF_tau_lim = [150.0 150.0 150.0];  % HP, K
        LH_tau_lim = [150.0 150.0 150.0];  % HP, K
        RF_tau_lim = [150.0 150.0 150.0];  % HP, K
        RH_tau_lim = [150.0 150.0 150.0];  % HP, K

    end
    methods
        function self = init(~, self)
            self.totalmass = 80;
            % torque limits for each leg 
            % HR = Hip roll
            % HP = Hip pitch
            % K = Knee 
            % W = wheel
    
            self.LF_tau_lim = [150.0 150.0 150.0];  % HR, HP, K
            self.LH_tau_lim = [150.0 150.0 150.0];  % HR, HP, K
            self.RF_tau_lim = [150.0 150.0 150.0];  % HR, HP, K
            self.RH_tau_lim = [150.0 150.0 150.0];  % HR, HP, K
        end
    end
end
