classdef Controller < handle

    properties
        mDt;
        mKp;
        mKi;
        mKd;
        mSetpoint;
        mPreviousError;
        mITerm;        
        mPTerm;
        mWindupMaxI;

    end

    methods
        function self = Controller(aDt, aKp, aKi, aKd, aSetpoint, aWindupMaxI)
            self.mDt = aDt;
            self.mKp = aKp;
            self.mKi = aKi;
            self.mKd = aKd;
            self.mSetpoint = aSetpoint;

            self.mPreviousError = 0;
            self.mITerm = 0;
            self.mPTerm = 0;
            self.mWindupMaxI = aWindupMaxI;

        end
        
        function effort = update(self, aCurrentValue)
            error = self.mSetpoint - aCurrentValue;
            error = wrapTo180(error);

            self.mPTerm = self.mKp * error;
            DTerm = self.mKd * (error - self.mPreviousError)/self.mDt;
            self.mITerm = self.mITerm + self.mKi * error * self.mDt;

            self.antiWindup();

            self.mPreviousError = error;
            disp("Terms")
            disp(self.mPTerm);
            disp(self.mITerm);
            effort = self.mPTerm + DTerm + self.mITerm;
        end

        function setPID(self, aP, aI, aD)
            self.mKp = aP;
            self.mKI = aI;
            self.mKd = aD;
        end

        function setSetpoint(self,aSetpoint)
            self.mSetpoint = aSetpoint;
            self.mPreviousError = 0;
            self.mITerm = 0;
        end

        function antiWindup(self)
            if self.mWindupMaxI ~= 0
                if self.mITerm > self.mWindupMaxI
                    self.mITerm = self.mWindupMaxI;
                elseif self.mITerm < -self.mWindupMaxI
                    self.mITerm = -self.mWindupMaxI;
                end 
            end

        end
    end
end    