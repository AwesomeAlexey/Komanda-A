#include <control_system/sliding_controller.h>
#include <gtest/gtest.h>
#include <cppmisc/traces.h>
#include <utils/utils.h>

std::string config_path;

struct SimData {
  std::vector<double> t;
  std::vector<double> theta;
  std::vector<double> phi;
  std::vector<double> dtheta;
  std::vector<double> dphi;
  std::vector<double> u;
};

template <>
struct Converter<SimData>
{
  static SimData doit(Json::Value const& json)
  {
    SimData data;
    json_get(json, "t", data.t);
    json_get(json, "theta", data.theta);
    json_get(json, "phi", data.phi);
    json_get(json, "dtheta", data.dtheta);
    json_get(json, "dphi", data.dphi);
    json_get(json, "u", data.u);
    return data;
  }
};

class SlidingControllerTest : 
    public testing::Test
{
protected:
  void SetUp() override
  {
    Json::Value cfg = json_load(config_path);
    controller.load(cfg);
    json_get(cfg, "simulation", simdata);
  }

  SlidingController controller;
  SimData simdata;
};

TEST_F(SlidingControllerTest, verify_output)
{
  bool equal = all_equal(simdata.t.size(), simdata.theta.size(), simdata.phi.size(), 
                          simdata.dtheta.size(), simdata.dphi.size(), simdata.u.size());
  if (!equal)
    throw_runtime_error("simulation data is corrupted");

  const int n = simdata.t.size();
  for (int i = 0; i < n; ++ i)
  {
    double theta = simdata.theta[i], 
      phi = simdata.phi[i],
      dtheta = simdata.dtheta[i], 
      dphi = simdata.dphi[i],
      t = simdata.t[i];
    double u = controller.process(t, theta, phi, dtheta, dphi);
    EXPECT_NEAR(u, simdata.u[i], 1e-5);
  }
}

int main(int argc, char* argv[])
{
  if (argc == 1)
  {
    err_msg("usage: ", argv[0], " path/to/feedback/config.json");
    return -1;
  }
  if (argc < 1)
  {
    err_msg("usage: test_name path/to/feedback/config.json");
    return -1;
  }
  config_path = argv[1];
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
