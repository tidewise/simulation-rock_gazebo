#include <rock_gazebo/details.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>

#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <stdexcept>

using namespace std;
using namespace rock_gazebo;

RTT::TaskContext* details::instanciateTask(std::string const& prefix,
    sdf::ElementConstPtr task_sdf)
{
    string name = task_sdf->Get<string>("name");
    bool absolute = task_sdf->Get<bool>("absolute", false).first;

    if (name.empty()) {
        if (absolute) {
            throw std::invalid_argument(
                "cannot have absolute set to true on a task with no name"
            );
        }

        name = prefix;
    }
    else if (!absolute && !prefix.empty()) {
        name = prefix + "::" + name;
    }
    string model = task_sdf->Get<string>("model");
    string file = task_sdf->Get<string>("filename");

    auto component_loader = RTT::ComponentLoader::Instance();

    if (!file.empty()) {
        component_loader->loadLibrary(file);
    }

    RTT::TaskContext* task_context = component_loader->loadComponent(name, model);
    if (!task_context) {
        throw std::logic_error("rock-gazebo: failed to load task context " + name +
                               " of model " + model + "\n");
    }

    gzmsg << "rock-gazebo: created task " << name << " of model " << model << endl;
    return task_context;
}
RTT::base::ActivityInterface* details::setupTaskActivity(RTT::TaskContext* task)
{
    // Create and start sequential task activities
    RTT::extras::SlaveActivity* activity = new RTT::extras::SlaveActivity(task->engine());
    activity->start();

    // Export the component interface on CORBA to Ruby access the component
    RTT::corba::TaskContextServer::Create(task);
    gzmsg << "created CORBA server for " << task->provides()->getName() << endl;
#if RTT_VERSION_GTE(2, 8, 99)
    task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);
#else
    RTT::corba::CorbaDispatcher::Instance(task->ports(),
        ORO_SCHED_OTHER,
        RTT::os::LowestPriority);
#endif

    return activity;
}

gz::sim::Entity details::resolveSubmodelRecursive(gz::sim::Entity const& root,
    std::list<std::string> const& names,
    gz::sim::EntityComponentManager& ecm)
{
    auto context = root;
    for (auto const& n : names) {
        auto child = gz::sim::Model(context).ModelByName(ecm, n);
        if (child == gz::sim::kNullEntity) {
            throw std::invalid_argument("could not find child model " + n + " of " +
                                        gz::sim::scopedName(context, ecm, "::"));
        }

        context = child;
    }

    return context;
}

gz::sim::Entity details::resolveSubmodelRecursive(gz::sim::Entity const& root,
    std::string const& scoped_name,
    gz::sim::EntityComponentManager& ecm)
{
    return resolveSubmodelRecursive(root, splitScopedName(scoped_name), ecm);
}

std::list<std::string> details::splitScopedName(std::string const& scopedName)
{
    std::list<std::string> result;
    std::string::size_type delim = scopedName.find("::"), current = 0;
    while (delim != std::string::npos) {
        result.push_back(scopedName.substr(current, delim - current));
        current = delim + 2;
        delim = scopedName.find("::", current);
    }
    result.push_back(scopedName.substr(current));
    return result;
}
