#include "core/particle_to_json.h"

namespace {
	auto Vector2dToStdVec = [](const Vector2d& evec) -> std::array<double, 2> {
		return {evec(0), evec(1)};
	};
	auto Vector3dToStdVec = [](const Vector3d& evec) -> std::array<double, 3> {
		return {evec(0), evec(1), evec(2)};
	};
	auto Matrix2dToStdVec = [](const Matrix2d& evec) -> std::array<double, 4> {
		return {evec(0,0), evec(0,1), evec(1,0), evec(1,1)};
	};
};

nlohmann::json particle_to_json(const Particle& p) {
    std::vector<std::array<double, 2>> landmark_poses(p.xf().size());
    const auto& xf = p.xf();
    std::transform(xf.begin(), xf.end(), landmark_poses.begin(), Vector2dToStdVec);

    std::vector<std::array<double, 4>> landmark_covs(p.Pf().size());
    const auto& Pf = p.Pf();
    std::transform(Pf.begin(), Pf.end(), landmark_covs.begin(), Matrix2dToStdVec);

    /*
    std::vector<nlohmann::json> landmarks(landmark_poses.size());
    std::transform(landmark_poses.begin(), landmark_poses.end(),
            landmark_covs.begin(), landmarks.begin(),
            [](auto pose, auto cov) {return nlohmann::json{{"pose", pose}, {"covariance",cov}};});
            */

    return {
        {"weight", p.w()},
        {"pose", Vector3dToStdVec(p.xv())},
        {"landmark_poses", landmark_poses},
        {"landmark_covs", landmark_covs}
    };
}

nlohmann::json particle_poses_to_json(const std::vector<Particle>& p_vec) {
    std::vector<std::array<double,3>> poses(p_vec.size());
    std::transform(p_vec.begin(), p_vec.end(),
		   poses.begin(),
		   [](auto p){return Vector3dToStdVec(p.xv());});
    return {
        {"poses", poses}
    };
}
