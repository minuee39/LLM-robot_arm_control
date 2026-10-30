// Keep plugin code alive while ROS callback-group weak references are destroyed.
// Scoped by launch LD_PRELOAD to this robot's MoveIt 2.12.4 move_group only.
#include <dlfcn.h>
#include <initializer_list>
#include <cstdio>
#include <cstdlib>

namespace
{
struct PluginCodeLifetime
{
  PluginCodeLifetime()
  {
    // Eagerly load both libraries so pluginlib's later dlclose cannot unmap code
    // used by weak_ptr control blocks still owned by rclcpp callback groups.
    // RTLD_NODELETE retains code until process exit; plugin objects, ROS nodes
    // and threads still run their ordinary destructors and shutdown handlers.
    for (const char* library : { "libmoveit_simple_controller_manager.so.2.12.4",
                                 "libmoveit_move_group_default_capabilities.so.2.12.4" })
    {
      void* handle = dlopen(library, RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
      if (!handle)
      {
        std::fprintf(stderr, "[pallet_shutdown_fix] Cannot retain %s: %s\n", library, dlerror());
        std::abort();
      }
      dlclose(handle);  // balance the handle; NODELETE retains the code mapping
    }
    std::fprintf(stderr, "[pallet_shutdown_fix] MoveIt 2.12.4 controller/capability code retained through ROS teardown\n");
  }
};
PluginCodeLifetime plugin_code_lifetime;
}  // namespace
