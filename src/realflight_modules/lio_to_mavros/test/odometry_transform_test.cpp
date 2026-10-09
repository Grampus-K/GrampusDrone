#include <gtest/gtest.h>
#include "lio_to_mavros/odometry_guard.hpp"
#include <random>
#include <limits>

using namespace lio_to_mavros;
namespace {
Rotation rpy(double r,double p,double y) {
  return (Eigen::AngleAxisd(y,Vector3::UnitZ()) * Eigen::AngleAxisd(p,Vector3::UnitY()) *
          Eigen::AngleAxisd(r,Vector3::UnitX())).toRotationMatrix();
}
Transform mounting() {
  Transform b_l,i_l; b_l.translation=Vector3(0,0,0.07);
  i_l.translation=Vector3(-0.011,-0.02329,0.04412);
  return imuFromBase(b_l,i_l);
}
void near(const Eigen::MatrixXd &a,const Eigen::MatrixXd &b,double tolerance=1e-10) {
  EXPECT_LT((a-b).norm(),tolerance);
}
}

TEST(Transform, IdentityAndTranslation) {
  Sample s; s.pose.translation=Vector3(1,2,3); s.linear=Vector3(4,5,6);
  auto out=transformSample(s,Transform(),Transform());
  near(out.pose.translation,s.pose.translation); near(out.linear,s.linear);
  near(out.pose_covariance,s.pose_covariance);
  Transform i_b; i_b.translation=Vector3(0,0,-0.07);
  out=transformSample(s,i_b,Transform()); near(out.pose.translation,Vector3(1,2,2.93));
}
TEST(Transform, ExtrinsicCompositionAndInverse) {
  near(mounting().translation,Vector3(-0.011,-0.02329,-0.02588));
  Transform b_l,i_l; b_l.rotation=rpy(.3,-.2,1); b_l.translation=Vector3(.1,-.2,.07);
  i_l.rotation=rpy(-.1,.4,-.2); i_l.translation=Vector3(-.011,-.02329,.04412);
  const auto i_b=imuFromBase(b_l,i_l);
  near((i_b*b_l).rotation,i_l.rotation); near((i_b*b_l).translation,i_l.translation);
  near((i_b*i_b.inverse()).rotation,Rotation::Identity());
  near((i_b*i_b.inverse()).translation,Vector3::Zero());
}
TEST(Transform, YawRollPitchAndBodyTwist) {
  Sample s; s.pose.rotation=rpy(.3,-.5,1.5707963267948966);
  s.pose.translation=Vector3(2,1,3); s.linear=Vector3(1,0,0); s.angular=Vector3(.1,.2,.3);
  Transform a_w; a_w.rotation=rpy(0,0,1.5707963267948966); a_w.translation=Vector3(-1,2,0);
  auto out=transformSample(s,Transform(),a_w);
  near(out.pose.translation,Vector3(-2,4,3)); near(out.pose.rotation,a_w.rotation*s.pose.rotation);
  near(out.linear,s.linear); near(out.angular,s.angular);
  Transform i_b; i_b.rotation=rpy(.1,.2,.5);
  out=transformSample(s,i_b,a_w);
  near(out.linear,i_b.rotation.transpose()*s.linear);
  near(out.angular,i_b.rotation.transpose()*s.angular);
}
TEST(Transform, LeverArmVelocitySign) {
  Sample s; s.angular=Vector3(0,0,2);
  auto out=transformSample(s,mounting(),Transform());
  near(out.linear,Vector3(.04658,-.022,0));
  // The IMU velocity for rotation about a stationary B must cancel exactly.
  s.linear=-s.angular.cross(mounting().translation);
  out=transformSample(s,mounting(),Transform()); near(out.linear,Vector3::Zero());
}
TEST(Transform, QuaternionDoubleCoverAndInvalidInputs) {
  Eigen::Quaterniond q(rpy(.3,.4,.5)), negative=q; negative.coeffs()*=-1;
  near(checkedQuaternion(q).toRotationMatrix(),checkedQuaternion(negative).toRotationMatrix());
  EXPECT_THROW(checkedQuaternion(Eigen::Quaterniond(0,0,0,0)),std::invalid_argument);
  Sample s; s.linear.x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(transformSample(s,Transform(),Transform()),std::invalid_argument);
  s=Sample(); s.pose_covariance(0,0)=-1;
  EXPECT_THROW(transformSample(s,Transform(),Transform()),std::invalid_argument);
  s=Sample(); s.pose_covariance(0,1)=.1;
  EXPECT_THROW(transformSample(s,Transform(),Transform()),std::invalid_argument);
}
TEST(Transform, ReferencePreservesTiltAndOnlyRotatesYaw) {
  Transform pose; pose.rotation=rpy(.3,-.2,.9); pose.translation=Vector3(3,4,5);
  auto a_w=referenceTransform(pose,.2,true,true);
  auto output=a_w*pose;
  near(output.translation,Vector3::Zero()); near(output.rotation,rpy(.3,-.2,.2));
  a_w=referenceTransform(pose,.4,false,false);
  near(a_w.translation,Vector3::Zero()); near(a_w.rotation,rpy(0,0,.4));
}
TEST(Covariance, PoseJacobianFiniteDifference) {
  Sample s; s.pose.rotation=rpy(.4,-.3,1.2); s.pose.translation=Vector3(1,2,3);
  Transform i_b=mounting(),a_w; i_b.rotation=rpy(.2,.3,-.4); a_w.rotation=rpy(0,0,-.6);
  auto baseline=transformSample(s,i_b,a_w); Matrix6 numeric;
  const double epsilon=1e-6;
  for(int i=0;i<6;++i) {
    Sample perturbed=s;
    if(i<3) perturbed.pose.translation(i)+=epsilon;
    else perturbed.pose.rotation=Eigen::AngleAxisd(epsilon,Vector3::Unit(i-3)).toRotationMatrix()*s.pose.rotation;
    const auto out=transformSample(perturbed,i_b,a_w);
    numeric.block<3,1>(0,i)=(out.pose.translation-baseline.pose.translation)/epsilon;
    const Eigen::AngleAxisd delta(out.pose.rotation*baseline.pose.rotation.transpose());
    numeric.block<3,1>(3,i)=delta.axis()*delta.angle()/epsilon;
  }
  near(numeric,poseJacobian(s.pose.rotation,i_b,a_w),1e-7);
}
TEST(Covariance, TwistJacobianFiniteDifference) {
  Sample s; s.linear=Vector3(.3,-.2,.8); s.angular=Vector3(1,-2,.5);
  Transform i_b=mounting(); i_b.rotation=rpy(.4,.3,-.9);
  const auto baseline=transformSample(s,i_b,Transform()); Matrix6 numeric;
  for(int i=0;i<6;++i) {
    Sample p=s; if(i<3) p.linear(i)+=1e-6; else p.angular(i-3)+=1e-6;
    const auto out=transformSample(p,i_b,Transform());
    numeric.block<3,1>(0,i)=(out.linear-baseline.linear)/1e-6;
    numeric.block<3,1>(3,i)=(out.angular-baseline.angular)/1e-6;
  }
  near(numeric,twistJacobian(i_b),1e-8);
}
TEST(Covariance, CrossTermsAndRandomPSD) {
  std::mt19937 generator(42); std::normal_distribution<double> normal;
  for(int n=0;n<100;++n) {
    Matrix6 m; for(int i=0;i<36;++i) m(i/6,i%6)=normal(generator);
    Sample s; s.pose_covariance=m*m.transpose(); s.twist_covariance=s.pose_covariance;
    s.pose.rotation=rpy(.3,.6,-.4); Transform a_w; a_w.rotation=rpy(0,0,.8);
    const auto out=transformSample(s,mounting(),a_w);
    EXPECT_GE(Eigen::SelfAdjointEigenSolver<Matrix6>(out.pose_covariance).eigenvalues().minCoeff(),-1e-10);
    EXPECT_GE(Eigen::SelfAdjointEigenSolver<Matrix6>(out.twist_covariance).eigenvalues().minCoeff(),-1e-10);
    const auto recovered=transformSample(out,mounting().inverse(),a_w.inverse());
    near(recovered.pose_covariance,s.pose_covariance,1e-9);
    near(recovered.twist_covariance,s.twist_covariance,1e-9);
    EXPECT_GT((out.twist_covariance.block<3,3>(0,3).norm()),1e-5);
  }
}

namespace {
struct GuardFixture : testing::Test {
  GuardOptions options;
  Sample input,output;
  Transform i_b;
  GuardFixture() { options.stable_seconds=.2; }
  void open(OdometryGuard &g) {
    EXPECT_FALSE(g.accept(input,100000000000ULL,1,100,0,i_b,output));
    EXPECT_FALSE(g.accept(input,100100000000ULL,2,100.1,.1,i_b,output));
    EXPECT_TRUE(g.accept(input,100300000000ULL,3,100.3,.3,i_b,output));
    EXPECT_TRUE(g.ready());
  }
};
}
TEST_F(GuardFixture, NewFramesOnlyAndNoRecovery) {
  OdometryGuard g(options); open(g);
  EXPECT_FALSE(g.accept(input,100300000000ULL,3,100.4,.4,i_b,output));
  EXPECT_TRUE(g.ready());
  g.tick(100.81,.81); EXPECT_TRUE(g.failed());
  EXPECT_FALSE(g.accept(input,100900000000ULL,4,100.9,.9,i_b,output));
}
TEST_F(GuardFixture, ChangedDuplicateAndBackwardsTime) {
  OdometryGuard g(options); open(g); input.angular.x()=.01;
  EXPECT_FALSE(g.accept(input,100300000000ULL,3,100.31,.31,i_b,output)); EXPECT_TRUE(g.failed());
  input=Sample(); OdometryGuard h(options); open(h);
  EXPECT_FALSE(h.accept(input,100200000000ULL,4,100.4,.4,i_b,output)); EXPECT_TRUE(h.failed());
}
TEST_F(GuardFixture, SequenceRestartAndPoseReset) {
  OdometryGuard g(options); open(g);
  EXPECT_FALSE(g.accept(input,100400000000ULL,0,100.4,.4,i_b,output)); EXPECT_TRUE(g.failed());
  OdometryGuard h(options); open(h); input.pose.translation.x()=10;
  EXPECT_FALSE(h.accept(input,100400000000ULL,4,100.4,.4,i_b,output)); EXPECT_TRUE(h.failed());
}
TEST_F(GuardFixture, ClockJumpAndAgeLimits) {
  OdometryGuard g(options); open(g); g.tick(102,.4); EXPECT_TRUE(g.failed());
  OdometryGuard old(options); EXPECT_FALSE(old.accept(input,99000000000ULL,1,100,0,i_b,output)); EXPECT_TRUE(old.failed());
  OdometryGuard future(options); EXPECT_FALSE(future.accept(input,100100000000ULL,1,100,0,i_b,output)); EXPECT_TRUE(future.failed());
}
TEST_F(GuardFixture, StartupGapDoesNotCountAsStability) {
  OdometryGuard g(options);
  EXPECT_FALSE(g.accept(input,100000000000ULL,1,100,0,i_b,output));
  EXPECT_FALSE(g.accept(input,101000000000ULL,2,101,1,i_b,output));
  EXPECT_FALSE(g.ready()); EXPECT_FALSE(g.failed());
}
TEST_F(GuardFixture, RotatingDoesNotOpenAndInvalidCovarianceLatches) {
  OdometryGuard g(options); input.angular.z()=.4;
  for(int i=0;i<5;++i) EXPECT_FALSE(g.accept(input,100000000000ULL+i*100000000ULL,i,100+.1*i,.1*i,i_b,output));
  EXPECT_FALSE(g.ready()); input.pose_covariance(0,0)=-1;
  EXPECT_FALSE(g.accept(input,100500000000ULL,5,100.5,.5,i_b,output)); EXPECT_TRUE(g.failed());
}
TEST(GuardOptions, RejectInvalidThresholds) {
  GuardOptions o; o.timeout=0; EXPECT_THROW(OdometryGuard g(o),std::invalid_argument);
  o=GuardOptions(); o.max_speed=std::numeric_limits<double>::infinity();
  EXPECT_THROW(OdometryGuard g(o),std::invalid_argument);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
