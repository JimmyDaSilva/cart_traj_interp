
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libcart_traj_interp-<target>.so
# and installs in the directory lib/orocos/cart_traj_interp/
#
orocos_component(cart_traj_interp cart_traj_interp-component.hpp cart_traj_interp-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/cart_traj_interp/ directory
#
orocos_install_headers(cart_traj_interp-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

