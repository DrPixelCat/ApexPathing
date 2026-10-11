package drivetrains.models;

import drivetrains.Mecanum;
import follower.FollowerConstants;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Vector;
import org.json.JSONObject;
import org.junit.Test;
import static org.junit.Assert.*;

public class DirectionalKinematicsTest {
    private FollowerConstants settings() throws Exception {
        FollowerConstants c=FollowerConstants.fromJson(new JSONObject().put("drivetrainType","OMNI"));
        c.forwardVelLimitIn=60; c.strafeVelLimitIn=45;
        c.forwardAccelLimitIn=100; c.strafeAccelLimitIn=70;
        c.angularVelLimitRad=8; c.angularAccelLimitRad=20;
        return c;
    }
    @Test public void defaultMecanumPreservesInterpolatedDirectionalLutAndLiveUpdates() throws Exception {
        FollowerConstants c=settings();
        DirectionalKinematics provider=DirectionalKinematics.mecanum(c);
        for(int pass=0;pass<2;pass++) {
            Mecanum.DirectionalLut expected=new Mecanum.DirectionalLut(c.forwardVelLimitIn,c.forwardAccelLimitIn,
                    c.strafeVelLimitIn,c.strafeAccelLimitIn);
            for(double degrees=-450;degrees<=450;degrees+=7.25) {
                Angle angle=Angle.fromDeg(degrees);
                Mecanum.DirectionalLut.DirectionalKinematics old=expected.getKinematics(
                        Vector.of(Math.cos(angle.getRad()),Math.sin(angle.getRad()),DistUnit.IN),Angle.zero());
                assertEquals(old.maxVel,provider.limitsAt(angle).maxVelocity,1e-10);
                assertEquals(old.maxAccel,provider.limitsAt(angle).maxAcceleration,1e-10);
            }
            c.forwardVelLimitIn=30; c.strafeAccelLimitIn=40;
        }
    }
    @Test public void omniLimitsFollowEveryWheelProjectionAndMountingOffset() throws Exception {
        FollowerConstants c=settings();
        for(int n:new int[]{3,4,5,6,9,17}) for(double offset:new double[]{0,.3,Math.PI/2}) {
            double[] x=new double[n],y=new double[n];
            for(int i=0;i<n;i++) { x[i]=-Math.sin(offset+i*2*Math.PI/n); y[i]=Math.cos(offset+i*2*Math.PI/n); }
            WheelProjectionModel wheels=new WheelProjectionModel(x,y,1);
            DirectionalKinematics provider=DirectionalKinematics.projected(c,wheels);
            double maxX=0,maxY=0;
            for(int i=0;i<n;i++) { maxX=Math.max(maxX,Math.abs(x[i])); maxY=Math.max(maxY,Math.abs(y[i])); }
            for(double theta=0;theta<2*Math.PI;theta+=.13) {
                double factor=0;
                for(int i=0;i<n;i++) factor=Math.max(factor,Math.abs(x[i]*Math.cos(theta)+y[i]*Math.sin(theta)));
                DirectionalLimits actual=provider.limitsAt(Angle.fromRad(theta));
                assertEquals(Math.min(60*maxX,45*maxY)/factor,actual.maxVelocity,1e-10);
                assertEquals(Math.min(100*maxX,70*maxY)/factor,actual.maxAcceleration,1e-10);
            }
        }
    }
    @Test public void customProviderIsRebuiltForSnapshotsAndNotCachedByDefault() throws Exception {
        FollowerConstants c=settings();
        HolonomicDriveModel live=DriveModels.mecanum(c).withDirectionalKinematics(snapshot -> angle ->
                new DirectionalLimits(snapshot.forwardVelLimitIn*(1-.2*Math.pow(Math.sin(angle.getRad()),2)),
                        snapshot.forwardAccelLimitIn));
        DriveModel frozen=live.withConstants(c.copy());
        c.forwardVelLimitIn=30;
        assertEquals(24,live.directionalLimits(Angle.fromDeg(90)).maxVelocity,1e-12);
        assertEquals(48,frozen.directionalLimits(Angle.fromDeg(90)).maxVelocity,1e-12);
        assertFalse(live.cacheSolverSamples());
        assertThrows(IllegalArgumentException.class,()->new DirectionalLimits(Double.NaN,10));
        assertThrows(IllegalArgumentException.class,()->new DirectionalLimits(10,-1));
    }
    @Test public void mecanumSubclassCanOverrideDirectionalLimitsWithoutReplacingItsModel() throws Exception {
        FollowerConstants c=settings();
        Mecanum drive=new Mecanum(new Mecanum.Constants(),null) {
            @Override protected DirectionalKinematics createDirectionalKinematics(FollowerConstants settings) {
                return direction -> new DirectionalLimits(settings.forwardVelLimitIn/3,settings.forwardAccelLimitIn/4);
            }
        };
        DriveModel model=DriveModels.forDrivetrain(drive,c,paths.movements.Path.PathType.HOLONOMIC);
        assertEquals(20,model.directionalLimits(Angle.fromDeg(45)).maxVelocity,1e-12);
        assertEquals(25,model.directionalLimits(Angle.fromDeg(45)).maxAcceleration,1e-12);
        assertFalse(model.cacheSolverSamples());
    }
    @Test public void stoppingHorizonIncludesCustomWeakDirectionsOutsideTheAxes() throws Exception {
        FollowerConstants c=settings(); c.translationalKV=.01; c.translationalKA=.001;
        DriveModel model=DriveModels.swerve(c).withDirectionalKinematics(snapshot -> direction ->
                new DirectionalLimits(60,Math.cos(direction.getRad()-Math.PI/4)>.999 ? 5 : 100));
        geometry.GeometryFactory g=new geometry.GeometryFactory();
        paths.movements.Path path=g.holonomicPath(g.pose(),g.pose(100,0)).setDriveModel(model).quickBuild();
        assertTrue(trajectory.StoppingHorizon.distance(c,path,1,model)>=1.5*60*60/(2*5)+.05*60-1e-5);
    }
    @Test public void storedThreeWheelSettingsMigrateToOmni() throws Exception {
        assertEquals(drivetrains.BaseDrivetrain.DrivetrainType.OMNI,
                FollowerConstants.fromJson(new JSONObject().put("drivetrainType","KIWI")).drivetrainType);
    }
}
