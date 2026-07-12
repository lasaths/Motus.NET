#include "motus_ompl.h"

#ifdef MOTUS_HAS_OMPL

#include <ompl/base/MotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#if __has_include(<ompl/geometric/planners/informedtrees/AITstar.h>)
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#define MOTUS_HAS_AITSTAR 1
#endif

#if __has_include(<ompl/geometric/planners/informedtrees/EITstar.h>)
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#define MOTUS_HAS_EITSTAR 1
#endif

#if __has_include(<ompl/geometric/planners/rrt/AORRTC.h>)
#include <ompl/geometric/planners/rrt/AORRTC.h>
#define MOTUS_HAS_AORRTC 1
#endif

#if __has_include(<ompl/geometric/planners/informedtrees/BITstar.h>)
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#define MOTUS_HAS_BLITSTAR 1
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;

extern void motus_set_last_error(const char*);

static bool IsValid(const ob::State* state, int dims, motus_ompl_validity_fn validity, void* userdata)
{
    if (!validity) return true;
    const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
    return validity(rv->values, dims, userdata) != 0;
}

static bool MotionValid(
    const ob::State* s1, const ob::State* s2, int dims, double step_size,
    motus_ompl_validity_fn validity, motus_ompl_motion_validity_fn motion_validity, void* userdata)
{
    const auto* rv1 = s1->as<ob::RealVectorStateSpace::StateType>();
    const auto* rv2 = s2->as<ob::RealVectorStateSpace::StateType>();
    if (motion_validity)
        return motion_validity(rv1->values, rv2->values, dims, userdata) != 0;

    double maxDelta = 0.0;
    for (int i = 0; i < dims; ++i)
        maxDelta = std::max(maxDelta, std::abs(rv2->values[i] - rv1->values[i]));
    const int steps = std::max(1, static_cast<int>(std::ceil(maxDelta / std::max(step_size, 1e-9))));
    for (int s = 0; s <= steps; ++s)
    {
        const double alpha = static_cast<double>(s) / steps;
        double q[64];
        if (dims > 64) return false;
        for (int i = 0; i < dims; ++i)
            q[i] = rv1->values[i] + alpha * (rv2->values[i] - rv1->values[i]);
        if (validity && validity(q, dims, userdata) == 0)
            return false;
    }
    return true;
}

class MotusMotionValidator : public ob::MotionValidator
{
public:
    MotusMotionValidator(
        const ob::SpaceInformationPtr& si,
        int dims,
        double step_size,
        motus_ompl_validity_fn validity,
        motus_ompl_motion_validity_fn motion_validity,
        void* userdata)
        : ob::MotionValidator(si)
        , dims_(dims)
        , step_size_(step_size)
        , validity_(validity)
        , motion_validity_(motion_validity)
        , userdata_(userdata)
    {}

    bool checkMotion(const ob::State* s1, const ob::State* s2) const override
    {
        return MotionValid(s1, s2, dims_, step_size_, validity_, motion_validity_, userdata_);
    }

    bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const override
    {
        if (checkMotion(s1, s2))
            return true;
        if (lastValid.first != nullptr)
        {
            si_->copyState(lastValid.first, s1);
            lastValid.second = 0.0;
        }
        return false;
    }

private:
    int dims_;
    double step_size_;
    motus_ompl_validity_fn validity_;
    motus_ompl_motion_validity_fn motion_validity_;
    void* userdata_;
};

static int WritePath(const og::PathGeometric& path, int dims, int max_states, double* out_path, int* out_count)
{
    int written = 0;
    for (unsigned int i = 0; i < path.getStateCount(); ++i)
    {
        if (written >= max_states) break;
        const auto* st = path.getState(i);
        const auto* rv = st->as<ob::RealVectorStateSpace::StateType>();
        for (int j = 0; j < dims; ++j)
            out_path[written * dims + j] = rv->values[j];
        ++written;
    }
    *out_count = written;
    return written > 0 ? MOTUS_OMPL_OK : MOTUS_OMPL_ERR;
}

static ob::SpaceInformationPtr SetupOmplSi(
    int dims,
    const double* low,
    const double* high,
    double step_size,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(dims);
    ob::RealVectorBounds bounds(dims);
    for (int i = 0; i < dims; ++i)
    {
        bounds.setLow(i, low[i]);
        bounds.setHigh(i, high[i]);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([&](const ob::State* s) {
        return IsValid(s, dims, validity, validity_userdata);
    });
    si->setMotionValidator(std::make_shared<MotusMotionValidator>(
        si, dims, step_size, validity, motion_validity, validity_userdata));
    si->setStateValidityCheckingResolution(0.01);
    si->setup();
    return si;
}

static ob::PlannerStatus SolvePlanner(
    const ob::PlannerPtr& planner,
    int max_iterations,
    double max_plan_time_sec)
{
    if (max_plan_time_sec > 0.0)
        return planner->solve(ob::timedPlannerTerminationCondition(max_plan_time_sec));
    return planner->solve(ob::IterationTerminationCondition(
        static_cast<unsigned int>(std::max(1, max_iterations))));
}

static ob::OptimizationObjectivePtr PathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

static ob::PlannerPtr CreatePlanner(
    int planner_id,
    const ob::SpaceInformationPtr& si,
    const std::shared_ptr<ob::ProblemDefinition>& pdef,
    double step_size,
    double goal_bias)
{
    switch (planner_id)
    {
    case MOTUS_OMPL_RRT_STAR:
    {
        auto planner = std::make_shared<og::RRTstar>(si);
        pdef->setOptimizationObjective(PathLengthObjective(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        planner->setRange(step_size);
        return planner;
    }
    case MOTUS_OMPL_AORRTC:
#ifdef MOTUS_HAS_AORRTC
    {
        auto planner = std::make_shared<og::AORRTC>(si);
        pdef->setOptimizationObjective(PathLengthObjective(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        planner->setRange(step_size);
        return planner;
    }
#else
        return nullptr;
#endif
    case MOTUS_OMPL_LBKPIECE:
    {
        auto planner = std::make_shared<og::LBKPIECE1>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();
        planner->setRange(step_size);
        return planner;
    }
    case MOTUS_OMPL_AIT_STAR:
#ifdef MOTUS_HAS_AITSTAR
    {
        auto planner = std::make_shared<og::AITstar>(si);
        pdef->setOptimizationObjective(PathLengthObjective(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        return planner;
    }
#else
        return nullptr;
#endif
    case MOTUS_OMPL_EIT_STAR:
#ifdef MOTUS_HAS_EITSTAR
    {
        auto planner = std::make_shared<og::EITstar>(si);
        pdef->setOptimizationObjective(PathLengthObjective(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        return planner;
    }
#else
        return nullptr;
#endif
    case MOTUS_OMPL_BLIT_STAR:
#ifdef MOTUS_HAS_BLITSTAR
    {
        auto planner = std::make_shared<og::BITstar>(si);
        pdef->setOptimizationObjective(PathLengthObjective(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        return planner;
    }
#else
        return nullptr;
#endif
    case MOTUS_OMPL_RRT_CONNECT:
    default:
    {
        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();
        planner->setRange(step_size);
#if defined(OMPL_VERSION) && OMPL_VERSION >= 0x0106000
        planner->setGoalBias(std::clamp(goal_bias, 0.0, 1.0));
#else
        (void)goal_bias;
#endif
        return planner;
    }
    }
}

static bool PlannerCompiled(int planner_id)
{
    switch (planner_id)
    {
    case MOTUS_OMPL_RRT_CONNECT:
    case MOTUS_OMPL_RRT_STAR:
    case MOTUS_OMPL_LBKPIECE:
        return true;
    case MOTUS_OMPL_AIT_STAR:
#ifdef MOTUS_HAS_AITSTAR
        return true;
#else
        return false;
#endif
    case MOTUS_OMPL_EIT_STAR:
#ifdef MOTUS_HAS_EITSTAR
        return true;
#else
        return false;
#endif
    case MOTUS_OMPL_AORRTC:
#ifdef MOTUS_HAS_AORRTC
        return true;
#else
        return false;
#endif
    case MOTUS_OMPL_BLIT_STAR:
#ifdef MOTUS_HAS_BLITSTAR
        return true;
#else
        return false;
#endif
    default:
        return false;
    }
}

extern "C" {

int motus_ompl_is_available(void) { return 1; }

int motus_ompl_planner_available(int planner_id)
{
    return PlannerCompiled(planner_id) ? 1 : 0;
}

int motus_ompl_plan(
    int dims,
    const double* low,
    const double* high,
    const double* start,
    const double* goal,
    int max_iterations,
    double max_plan_time_sec,
    double step_size,
    double goal_bias,
    int planner_id,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    if (!low || !high || !start || !goal || !out_path || !out_count || dims <= 0 || max_states <= 0)
    {
        motus_set_last_error("invalid arguments");
        return MOTUS_OMPL_ERR;
    }

    *out_count = 0;

    if (!PlannerCompiled(planner_id))
    {
        motus_set_last_error("planner not available in this OMPL build");
        return MOTUS_OMPL_ERR;
    }

    auto si = SetupOmplSi(dims, low, high, step_size, validity, motion_validity, validity_userdata);

    ob::ScopedState<> startState(si->getStateSpace());
    ob::ScopedState<> goalState(si->getStateSpace());
    for (int i = 0; i < dims; ++i)
    {
        startState[i] = start[i];
        goalState[i] = goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(startState, goalState);

    auto planner = CreatePlanner(planner_id, si, pdef, step_size, goal_bias);
    if (!planner)
    {
        motus_set_last_error("planner not available in this OMPL build");
        return MOTUS_OMPL_ERR;
    }

    const auto solved = SolvePlanner(planner, max_iterations, max_plan_time_sec);
    if (!solved)
    {
        motus_set_last_error("planner failed");
        return MOTUS_OMPL_ERR;
    }

    auto pathBase = pdef->getSolutionPath();
    if (!pathBase)
    {
        motus_set_last_error("no solution path");
        return MOTUS_OMPL_ERR;
    }

    auto path = std::static_pointer_cast<og::PathGeometric>(pathBase);
    const auto stateCount = path->getStateCount();
    path->interpolate(static_cast<unsigned int>(
        std::min(static_cast<std::size_t>(max_states), stateCount)));

    return WritePath(*path, dims, max_states, out_path, out_count);
}

int motus_ompl_rrt_connect(
    int dims,
    const double* low,
    const double* high,
    const double* start,
    const double* goal,
    int max_iterations,
    double max_plan_time_sec,
    double step_size,
    double goal_bias,
    int planner_id,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    return motus_ompl_plan(
        dims, low, high, start, goal, max_iterations, max_plan_time_sec, step_size, goal_bias, planner_id,
        validity, motion_validity, validity_userdata, out_path, max_states, out_count);
}

int motus_ompl_simplify_path(
    int dims,
    const double* path,
    int path_count,
    double step_size,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    if (!path || path_count < 2 || !out_path || !out_count || dims <= 0)
        return MOTUS_OMPL_ERR;

    auto space = std::make_shared<ob::RealVectorStateSpace>(dims);
    ob::RealVectorBounds bounds(dims);
    for (int i = 0; i < dims; ++i)
    {
        bounds.setLow(i, -1e6);
        bounds.setHigh(i, 1e6);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([&](const ob::State* s) {
        return IsValid(s, dims, validity, validity_userdata);
    });
    si->setMotionValidator(std::make_shared<MotusMotionValidator>(
        si, dims, step_size, validity, motion_validity, validity_userdata));
    si->setup();

    og::PathGeometric geom(si);
    for (int i = 0; i < path_count; ++i)
    {
        ob::ScopedState<> st(space);
        for (int j = 0; j < dims; ++j)
            st[j] = path[i * dims + j];
        geom.append(st.get());
    }

    og::PathSimplifier simplifier(si);
    simplifier.simplifyMax(geom);

    return WritePath(geom, dims, max_states, out_path, out_count);
}

} // extern "C"

#endif
