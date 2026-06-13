#include "common/eigen_types.h"
#include "common/std_types.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "core/types/edge_se3.h"
#include "core/types/vertex_se3.h"

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

namespace {

lightning::SE3 PoseX(double x) {
    return lightning::SE3(lightning::Quatd::Identity(), lightning::Vec3d(x, 0.0, 0.0));
}

bool ContainsEdge(const std::vector<lightning::miao::Edge*>& edges, const lightning::miao::Edge* edge) {
    return std::find(edges.begin(), edges.end(), edge) != edges.end();
}

bool ContainsEdge(const lightning::miao::Graph::EdgeContainer& edges, const lightning::miao::Edge* edge) {
    return std::find_if(edges.begin(), edges.end(), [&](const auto& item) { return item.get() == edge; }) != edges.end();
}

bool ContainsEdge(const std::set<std::shared_ptr<lightning::miao::Edge>>& edges, const lightning::miao::Edge* edge) {
    return std::find_if(edges.begin(), edges.end(), [&](const auto& item) { return item.get() == edge; }) != edges.end();
}

double TranslationDelta(const lightning::SE3& a, const lightning::SE3& b) {
    return (a.translation() - b.translation()).norm();
}

std::shared_ptr<lightning::miao::EdgeSE3> MakeEdge(const std::shared_ptr<lightning::miao::VertexSE3>& v0,
                                                   const std::shared_ptr<lightning::miao::VertexSE3>& v1,
                                                   const lightning::SE3& measurement,
                                                   const lightning::Mat6d& information) {
    auto edge = std::make_shared<lightning::miao::EdgeSE3>();
    edge->SetVertex(0, v0);
    edge->SetVertex(1, v1);
    edge->SetMeasurement(measurement);
    edge->SetInformation(information);
    return edge;
}

}  // namespace

int main(int argc, char** argv) {
    (void)argc;
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::WARNING;

    using namespace lightning;

    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_DENSE, true);
    config.incremental_mode_ = true;
    auto optimizer = miao::SetupOptimizer<6, 3>(config);

    auto v0 = std::make_shared<miao::VertexSE3>();
    v0->SetId(0);
    v0->SetEstimate(PoseX(0.0));
    v0->SetFixed(true);
    optimizer->AddVertex(v0);

    auto v1 = std::make_shared<miao::VertexSE3>();
    v1->SetId(1);
    v1->SetEstimate(PoseX(1.0));
    optimizer->AddVertex(v1);

    auto v2 = std::make_shared<miao::VertexSE3>();
    v2->SetId(2);
    v2->SetEstimate(PoseX(2.0));
    optimizer->AddVertex(v2);

    const Mat6d odom_info = Mat6d::Identity() * 100.0;
    const Mat6d loop_info = Mat6d::Identity() * 100.0;
    optimizer->AddEdge(MakeEdge(v0, v1, PoseX(1.0), odom_info));
    optimizer->AddEdge(MakeEdge(v1, v2, PoseX(1.0), odom_info));

    optimizer->InitializeOptimization();
    optimizer->ComputeActiveErrors();
    optimizer->Optimize(5);
    optimizer->ComputeActiveErrors();
    const double baseline_chi2 = optimizer->ActiveChi2();
    if (!std::isfinite(baseline_chi2) || baseline_chi2 > 1e-6) {
        std::cerr << "baseline odom-only chi2 is too large: " << baseline_chi2 << "\n";
        return 1;
    }

    const SE3 snapshot_v1 = v1->Estimate();
    const SE3 snapshot_v2 = v2->Estimate();

    auto wrong_loop = MakeEdge(v0, v2, PoseX(5.0), loop_info);
    optimizer->AddEdge(wrong_loop);

    optimizer->InitializeOptimization();
    optimizer->ComputeActiveErrors();
    if (!ContainsEdge(optimizer->ActiveEdges(), wrong_loop.get())) {
        std::cerr << "wrong loop was not added to active_edges\n";
        return 1;
    }
    if (!ContainsEdge(optimizer->NewEdges(), wrong_loop.get())) {
        std::cerr << "wrong loop was not tracked in new_edges before optimize\n";
        return 1;
    }
    const double wrong_loop_error_before = wrong_loop->Chi2();
    if (!std::isfinite(wrong_loop_error_before) || wrong_loop_error_before < 1.0) {
        std::cerr << "wrong loop error is unexpectedly small before optimize: " << wrong_loop_error_before << "\n";
        return 1;
    }

    optimizer->Optimize(10);
    optimizer->ComputeActiveErrors();
    const SE3 trial_v1 = v1->Estimate();
    const SE3 trial_v2 = v2->Estimate();
    if (TranslationDelta(trial_v2, snapshot_v2) < 1e-4) {
        std::cerr << "trial optimize did not perturb the test graph\n";
        return 1;
    }

    if (!optimizer->RemoveEdge(wrong_loop)) {
        std::cerr << "RemoveEdge failed for wrong loop\n";
        return 1;
    }
    v1->SetEstimate(snapshot_v1);
    v2->SetEstimate(snapshot_v2);

    if (ContainsEdge(optimizer->ActiveEdges(), wrong_loop.get())) {
        std::cerr << "wrong loop still exists in active_edges after RemoveEdge\n";
        return 1;
    }
    if (ContainsEdge(optimizer->NewEdges(), wrong_loop.get())) {
        std::cerr << "wrong loop still exists in new_edges after RemoveEdge\n";
        return 1;
    }
    if (ContainsEdge(optimizer->GetEdges(), wrong_loop.get())) {
        std::cerr << "wrong loop still exists in graph edges after RemoveEdge\n";
        return 1;
    }
    if (ContainsEdge(v0->GetEdges(), wrong_loop.get()) || ContainsEdge(v2->GetEdges(), wrong_loop.get())) {
        std::cerr << "wrong loop still exists in vertex edge lists after RemoveEdge\n";
        return 1;
    }

    optimizer->InitializeOptimization();
    optimizer->ComputeActiveErrors();
    optimizer->Optimize(5);
    optimizer->ComputeActiveErrors();
    const double after_remove_chi2 = optimizer->ActiveChi2();
    if (!std::isfinite(after_remove_chi2) || after_remove_chi2 > 1e-6) {
        std::cerr << "removed loop still appears to affect active error, chi2=" << after_remove_chi2 << "\n";
        return 1;
    }
    if (TranslationDelta(v1->Estimate(), snapshot_v1) > 1e-6 || TranslationDelta(v2->Estimate(), snapshot_v2) > 1e-6) {
        std::cerr << "vertex estimate was not restored after rejected no-commit\n";
        return 1;
    }
    if (TranslationDelta(trial_v1, v1->Estimate()) < 1e-4 && TranslationDelta(trial_v2, v2->Estimate()) < 1e-4) {
        std::cerr << "post-remove optimize kept the rejected trial estimate\n";
        return 1;
    }

    std::cout << "rejected_no_commit check passed\n";
    return 0;
}
