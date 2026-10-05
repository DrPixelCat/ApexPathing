package feedforward;

import org.json.JSONObject;

/** Measured shared wheel authority and chassis drag, in inches/radians/normalized motor power.
 * Combine chassis force first; invert each wheel using its reference velocity and signed power regime.
 * Calibration must be validated on combined motion; isolated axis fits alone cannot establish this model. */
public final class MecanumWheelModel {
    public final double gain, speedLoss, regenerativeBraking, neutralBraking;
    public final double forwardDrag, strafeDrag, angularDrag, effectiveRadius, maximumWheelSpeed, validationRms;
    public final double commandPreviewSeconds;
    public static final double MIN_POWER = .02;

    public MecanumWheelModel(double gain, double speedLoss, double regenerativeBraking, double neutralBraking,
                             double forwardDrag, double strafeDrag, double angularDrag, double effectiveRadius,
                             double maximumWheelSpeed, double validationRms) {
        this(gain,speedLoss,regenerativeBraking,neutralBraking,forwardDrag,strafeDrag,angularDrag,effectiveRadius,
                maximumWheelSpeed,validationRms,0);
    }
    public MecanumWheelModel(double gain, double speedLoss, double regenerativeBraking, double neutralBraking,
                             double forwardDrag, double strafeDrag, double angularDrag, double effectiveRadius,
                             double maximumWheelSpeed, double validationRms,double commandPreviewSeconds) {
        double[] values = {gain,speedLoss,regenerativeBraking,neutralBraking,forwardDrag,strafeDrag,angularDrag,
                effectiveRadius,maximumWheelSpeed,validationRms,commandPreviewSeconds};
        for (double value : values) if (!Double.isFinite(value) || value < 0) throw new IllegalArgumentException("Finite nonnegative wheel calibration required");
        if (gain <= 0 || forwardDrag <= 0 || strafeDrag <= 0 || effectiveRadius <= 0 || maximumWheelSpeed <= 0
                || gain-speedLoss*maximumWheelSpeed <= 0 || validationRms > .04 || commandPreviewSeconds>.05) {
            throw new IllegalArgumentException("Validated wheel authority and a positive measured domain required");
        }
        this.gain=gain; this.speedLoss=speedLoss; this.regenerativeBraking=regenerativeBraking; this.neutralBraking=neutralBraking;
        this.forwardDrag=forwardDrag; this.strafeDrag=strafeDrag; this.angularDrag=angularDrag;
        this.effectiveRadius=effectiveRadius; this.maximumWheelSpeed=maximumWheelSpeed; this.validationRms=validationRms;
        this.commandPreviewSeconds=commandPreviewSeconds;
    }

    /** Reusable solved wheel demand. Pulse debt belongs to an executor, never to the shared model. */
    public static final class Demand {
        public final double[] aligned = new double[4], opposing = new double[4], opposingDuty = new double[4];
        public double peak;
        public boolean insideDomain;
        public void pulse(double[] debt, double[] output) {
            for(int i=0;i<4;i++) {
                if(aligned[i]==opposing[i]) { output[i]=aligned[i]; debt[i]=.5; }
                else { debt[i]+=opposingDuty[i]; if(debt[i]>=1) { output[i]=opposing[i];debt[i]-=1; } else output[i]=aligned[i]; }
            }
        }
        public double mean(int i) { return aligned[i]*(1-opposingDuty[i])+opposing[i]*opposingDuty[i]; }
    }

    /** Measured chassis drag magnitude along combined linear motion. */
    public double dragMagnitude(double vx,double vy) {
        double mag=Math.hypot(vx,vy);
        return mag>1e-9?mag/(Math.abs(vx)/forwardDrag+Math.abs(vy)/strafeDrag):0;
    }
    /** Body acceleration is the field acceleration rotated into the body frame, not d(body velocity)/dt. */
    public void solve(double vx,double vy,double omega,double ax,double ay,double alpha,Demand output) {
        solve(vx,vy,omega,ax,ay,alpha,1,output);
    }
    private void solve(double vx,double vy,double omega,double ax,double ay,double alpha,double forceScale,Demand output) {
        double mag=Math.hypot(vx,vy), dx=0,dy=0;
        if(mag>1e-9) { dx=vx/mag;dy=vy/mag; }
        else { double acceleration=Math.hypot(ax,ay);if(acceleration>1e-9) { dx=ax/acceleration;dy=ay/acceleration; } }
        double drag=dragMagnitude(dx,dy);
        double forceX=(ax+drag*dx)*forceScale,forceY=(ay+drag*dy)*forceScale;
        double torque=(alpha+angularDrag*Math.signum(omega))*effectiveRadius*forceScale;
        output.peak=0;output.insideDomain=true;
        for(int i=0;i<4;i++) {
            int ys=i==0 || i==3?-1:1,ts=i==0 || i==2?-1:1;
            double w=vx+ys*vy+ts*effectiveRadius*omega, force=forceX+ys*forceY+ts*torque;
            double speed=Math.abs(w),sign=Math.signum(w);output.opposingDuty[i]=0;
            if(speed>maximumWheelSpeed+1e-6) output.insideDomain=false;
            if(speed<1e-7) { output.aligned[i]=output.opposing[i]=force/gain; }
            else {
                double authority=gain-speedLoss*speed;
                if(authority<=0) { output.insideDomain=false;output.aligned[i]=output.opposing[i]=Double.POSITIVE_INFINITY;continue; }
                double f=force*sign,aligned=f/authority,opposing=(f+regenerativeBraking*speed)/gain;
                if(aligned>=MIN_POWER) output.aligned[i]=output.opposing[i]=sign*aligned;
                else if(opposing<=-MIN_POWER) output.aligned[i]=output.opposing[i]=sign*opposing;
                else {
                    output.aligned[i]=sign*MIN_POWER;output.opposing[i]=-sign*MIN_POWER;
                    double high=authority*MIN_POWER,low=-gain*MIN_POWER-regenerativeBraking*speed;
                    output.opposingDuty[i]=Math.max(0,Math.min(1,(high-f)/(high-low)));
                }
            }
            output.peak=Math.max(output.peak,Math.max(Math.abs(output.aligned[i]),Math.abs(output.opposing[i])));
        }
        if(!output.insideDomain) output.peak=Double.POSITIVE_INFINITY;
    }

    /** Saturate in chassis-force space. Scaling already inverted powers can switch
     * a wheel between powered and regenerative regimes and invalidate its response. */
    public double solveBounded(double vx,double vy,double omega,double ax,double ay,double alpha,double powerLimit,Demand output) {
        if(!Double.isFinite(powerLimit) || powerLimit<MIN_POWER || powerLimit>1) throw new IllegalArgumentException("Wheel power limit must be in [.02, 1]");
        solve(vx,vy,omega,ax,ay,alpha,output);
        if (!output.insideDomain) { throw new IllegalArgumentException("Wheel demand is outside the calibrated domain"); }
        if (output.peak <= powerLimit) { return 1; }
        double low=0,high=1;
        for(int i=0;i<28;i++) {
            double scale=(low+high)/2;
            solve(vx,vy,omega,ax,ay,alpha,scale,output);
            if(output.peak<=powerLimit)low=scale;else high=scale;
        }
        solve(vx,vy,omega,ax,ay,alpha,low,output);return low;
    }

    /** Exact affine chassis-force bounds at a fixed speed; avoids numerical searches
     * inside every receding-horizon solver transition. Pulse gaps remain attainable. */
    public double maximumAcceleration(double bx,double by,double curvature,double v,double prime,double second,
                                      double min,double max,double limit) {
        if(limit<MIN_POWER || (v<1e-9 && min<0 && max>0)) return Double.NEGATIVE_INFINITY;
        double direction=v>1e-9 || max>0?1:min<0?-1:0;
        double drag=direction==0?0:1/(Math.abs(bx)/forwardDrag+Math.abs(by)/strafeDrag);
        double normal=curvature*v*v,omega=prime*v;
        double fx=direction*drag*bx-by*normal,fy=direction*drag*by+bx*normal;
        double torque=(second*v*v+angularDrag*Math.signum(omega))*effectiveRadius;
        for(int i=0;i<4;i++) {
            int ys=i==0 || i==3?-1:1,ts=i==0 || i==2?-1:1;
            double w=v*(bx+ys*by+ts*effectiveRadius*prime),speed=Math.abs(w);
            if(speed>maximumWheelSpeed+1e-6) return Double.NaN;
            double base=fx+ys*fy+ts*torque,slope=bx+ys*by+ts*effectiveRadius*prime;
            double lower=-gain*limit,upper=gain*limit;
            if(speed>1e-7) {
                double aligned=(gain-speedLoss*speed)*limit,opposing=-gain*limit-regenerativeBraking*speed;
                lower=w>0?opposing:-aligned;upper=w>0?aligned:-opposing;
            }
            if(Math.abs(slope)<1e-12) { if(base<lower-1e-9 || base>upper+1e-9)return Double.NaN; }
            else {
                double a=(lower-base)/slope,b=(upper-base)/slope;
                min=Math.max(min,Math.min(a,b));max=Math.min(max,Math.max(a,b));
            }
        }
        return max>=min-1e-9?max:Double.NaN;
    }

    public JSONObject toJson() {
        try { return new JSONObject().put("version",1).put("gain",gain).put("speedLoss",speedLoss)
                .put("regenerativeBraking",regenerativeBraking).put("neutralBraking",neutralBraking)
                .put("forwardDrag",forwardDrag).put("strafeDrag",strafeDrag).put("angularDrag",angularDrag)
                .put("effectiveRadius",effectiveRadius).put("maximumWheelSpeed",maximumWheelSpeed).put("validationRms",validationRms)
                .put("commandPreviewSeconds",commandPreviewSeconds); }
        catch(Exception e) { throw new IllegalStateException("Cannot save wheel characterization",e); }
    }
    public static MecanumWheelModel fromJson(JSONObject json) {
        try {
            if(json.getInt("version")!=1) throw new IllegalArgumentException("Unknown wheel calibration version");
            return new MecanumWheelModel(json.getDouble("gain"),json.getDouble("speedLoss"),json.getDouble("regenerativeBraking"),
                    json.getDouble("neutralBraking"),json.getDouble("forwardDrag"),json.getDouble("strafeDrag"),json.getDouble("angularDrag"),
                    json.getDouble("effectiveRadius"),json.getDouble("maximumWheelSpeed"),json.getDouble("validationRms"),
                    json.optDouble("commandPreviewSeconds",0));
        } catch(Exception e) { throw new IllegalArgumentException("Invalid wheel calibration",e); }
    }
}
