# Includes
# --------

include(GNUInstallDirs)

# Sanity Checks
# -------------

if(NOT EXISTS ${TARGET_PATH})
    message(FATAL_ERROR "`TARGET_PATH` not provided or does not exist.")
endif()

message(STATUS "InstallRos2 - finding and installing dependencies for ${TARGET_PATH}.")

# Settings
# --------

set(recursive 0) # setting to '1' causes warning messages to print when it tries to get prereqs for the found libraries.

if(INSTALL_SYSTEM_LIBS)
    set(excludeSystem 0) # setting to '0' will get all dependencies, even stdc++!
else()
    set(excludeSystem 1)
endif()

if(NOT DESTINATION)
    set(DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if(NO_INSTALL_RPATH)
    set(noInstallRpath NO_INSTALL_RPATH)
endif()

if(SEARCH_DIRS)
    set(searchDirs SEARCH_DIRS ${SEARCH_DIRS})
endif()

set(defaultRMW fastrtps)
if(NOT RMW)
    set(RMW ${defaultRMW})
endif()

function(install_prerequisites)
    message(FATAL_ERROR "You should implement this yourself!")
endfunction()

function(get_prerequisites)
    message(FATAL_ERROR "You should implement this yourself!")
endfunction()

# Converts a path into an absolute path
# This function can cause a FATAL_ERROR if the conversion would produce a file that does not exist.
#
# Parameters
# ----------
#
# ``path``
#   the path to convert
#
# ``roots``
#   List of directories to use as roots and prefix them to the relative path.
#   The first root producing an existing path will be used.
#
# Result Variables
# ----------------
#
# ``absolute_path``
#   Variable containing absolute path
#
function(_get_absolute_path path roots absolute_path)
    if (IS_ABSOLUTE ${path})
        # Nothing to do if the path is already absolute
        set(_absolute_path ${path})
    else()
        # The path is relative, try to make it absolute using provided roots
        foreach(root ${roots})
            set(maybe_absolute_path "${root}/${path}")
            if (EXISTS ${maybe_absolute_path})
                set(_absolute_path ${maybe_absolute_path})
                break()
            endif()
        endforeach()
    endif()

    if (_absolute_path AND EXISTS ${_absolute_path})
        set(${absolute_path} ${_absolute_path} PARENT_SCOPE)
    else()
        # When computing pre-requisites of pre-requisites we may get some relative paths that we are
        # unable to properly covert in absolute paths.
        # Rather than throwing a FATAL_ERROR we just print a warning and ignore the problem.
        # This is good for now, as these pre-requisites are already installed as part of other operations,
        # but it should be fixed: see ROS2-175
        message(WARNING "Absolute path for ${path} does not exist: ${_absolute_path}")
        set(${absolute_path} ${path} PARENT_SCOPE)
    endif()
endfunction()

# Gets the RMW implementation library and its dependencies.
# This function can cause a FATAL_ERROR if the RMW implementation is not found.
#
# Parameters
# ----------
#
# ``rmw_impl_name``
#   name of the RMW implementation that we want to get
#
# ``searchPath``
#   Directory where the RMW implementation library is expected to be found.
#   Note that transitive dependencies are not required to be in this directory.
#
# Result Variables
# ----------------
#
# ``depsVar``
#   Variable containing absolute paths to the found ROS libraries
#
function(_get_rmw_implementation_prereqs rmw_impl_name searchPath depsVar)

    # Generate the name of the RMW implementation library that we need to look for.
    set(rmw_implementation_lib_name "librmw_${rmw_impl_name}_cpp.so")

    # Now we search for it in the provided searchPath
    # We throw an error if this library is not found.
    set(rmw_implementation_lib_path "${searchPath}/${rmw_implementation_lib_name}")
    if (EXISTS ${rmw_implementation_lib_path})
        # We found the RMW implementation: add it as a runtime dependency
        list(APPEND _depsVar "${rmw_implementation_lib_path}")

        # It's not enough to add the RMW implementation, as it may introduce transitive requirements.
        # So we search for prerequisites of the RMW implementation.
        # The prerequisites are computed relative to `${rmw_implementation_lib_path}`, so we have to add the directory path 

        get_prerequisites(${rmw_implementation_lib_path} rmw_prereqs ${excludeSystem} ${recursive} "" "${searchDirs}" ${noInstallRpath})
        set(roots ${searchPath} ${SEARCH_DIRS} ${CMAKE_SOURCE_DIR})
        foreach(prereq ${rmw_prereqs})
            _get_absolute_path(${prereq} "${roots}" prereq_absolute_path)
            list(APPEND _depsVar ${prereq_absolute_path})
        endforeach()
    else()
        message(FATAL_ERROR "Not found RMW implementation library at ${rmw_implementation_lib_path}")
    endif()

    set(${depsVar} ${_depsVar} PARENT_SCOPE)
endfunction()

# Gets the RMW-specific implementation of a provided ROS interface library.
# This function can cause a FATAL_ERROR if the provided interface is valid,
# but its RMW-specific implementation is not found.
#
# Parameters
# ----------
#
# ``rmw_impl_name``
#   name of the RMW implementation that we want to get
#
# ``interface``
#   Absolute path to a ROS interface library.
#   If this is not a correct library, it will be ignored.
#
# Result Variables
# ----------------
#
# ``depsVar``
#   Variable containing absolute paths to the RMW-specific implementation library and its dependencies
#
function(_get_rmw_specific_interfaces rmw_impl_name interface depsVar)

    # Construct the path of the RMW-specific message library
    # This is guaranteed to work as RMW-agnostic and RMW-specific libraries are always in the same directory
    # as they are built from the same ROS package.

    if (interface MATCHES "typesupport" AND NOT interface MATCHES "${rmw_impl_name}" AND NOT interface MATCHES "typesupport_introspection")
        string(REPLACE "typesupport" "typesupport_${rmw_impl_name}" rmw_specific_interface ${interface})        
        if (EXISTS ${rmw_specific_interface})
            list(APPEND dependenciesVar ${rmw_specific_interface})
            # This new library may be introducing new dependencies
            get_prerequisites(${rmw_specific_interface} interface_prereqs ${excludeSystem} ${recursive} "" ${searchDirs} ${noInstallRpath})
            # Prerequisites may appear with relative path: fix them here
            get_filename_component(interfaceDir ${interface} DIRECTORY)
            set(roots ${interfaceDir} ${SEARCH_DIRS} ${CMAKE_SOURCE_DIR})
            foreach(prereq ${interface_prereqs})
                _get_absolute_path(${prereq} "${roots}" prereq_absolute_path)
                list(APPEND dependenciesVar ${prereq_absolute_path})
            endforeach()
        else()
            message(FATAL_ERROR "Not found RMW-specific library at ${rmw_specific_interface}")
        endif()

    endif()

    set(${depsVar} ${dependenciesVar} PARENT_SCOPE)
endfunction()

# Gets the rosidl introspection library of a provided ROS interface library.
#
# Parameters
# ----------
#
# ``rmw_impl_name``
#   name of the RMW implementation that we want to get
#
# ``interface``
#   Absolute path to a ROS interface library.
#   If this is not a correct library, it will be ignored.
#
# Result Variables
# ----------------
#
# ``depsVar``
#   Variable containing absolute paths to the typesupport introspection library and its dependencies
#
function(_get_introspection_libraries rmw_impl_name interface depsVar)
    # Construct the path of the introspection message library
    # This is guaranteed to work as interface and introspection libraries are always in the same directory
    # as they are built from the same ROS package.

    if (interface MATCHES "typesupport" AND NOT interface MATCHES "typesupport_introspection")
        if (interface MATCHES "${rmw_impl_name}")
            string(REPLACE "${rmw_impl_name}" "introspection" introspection_lib ${interface})
        else()
            string(REPLACE "typesupport" "typesupport_introspection" introspection_lib ${interface})
        endif()

        if (EXISTS ${introspection_lib})
            list(APPEND dependenciesVar ${introspection_lib})
            # This new library may be introducing new dependencies
            get_prerequisites(${introspection_lib} interface_prereqs ${excludeSystem} ${recursive} "" ${searchDirs} ${noInstallRpath})
            # Prerequisites may appear with relative path: fix them here
            get_filename_component(interfaceDir ${interface} DIRECTORY)
            set(roots ${interfaceDir} ${SEARCH_DIRS})
            foreach(prereq ${interface_prereqs})
                _get_absolute_path(${prereq} "${roots}" prereq_absolute_path)
                list(APPEND dependenciesVar ${prereq_absolute_path})
            endforeach()
        else()
            message(FATAL_ERROR "Not found introspection library at ${introspection_lib}")
        endif()
    endif()

    set(${depsVar} ${dependenciesVar} PARENT_SCOPE)
endfunction()


# Script
# --------

# First we need to find target prerequisites set during the build.
get_prerequisites(${TARGET_PATH} app_prereqs ${excludeSystem} ${recursive} "" ${searchDirs} ${noInstallRpath})

# Search ROS 2 RMW interface library in the current prerequisites.
# If a target does not link to this interface library it does not need RMW specific runtime libraries.
set(rmw_interface_lib_path "")
foreach(prereq ${app_prereqs})
    if (prereq MATCHES "librmw.so$")
        set(rmw_interface_lib_path "${prereq}")
    endif()
endforeach()

# This is where we will store all the dependencies we find
set(ros2_runtime_prereqs "")

# At the time of writing, the choice of the ROS 2 RMW implementation is the only factor that
# affects which libraries will be loaded via dlopen. New ROS 2 distribution may require additions.

if(rmw_interface_lib_path)

    foreach(rmw_impl_name ${RMW})
        # The block is organized in multiple parts: not all of them are required by every RMW
        # implementation.

        ##### Search ROS 2 RMW implementation library
        # Generate the name of the RMW implementation library that we need to look for.
        # Now we search for the RMW implementation library, assuming that it's in the same directory as the RMW interface.
        # This assumption is verified as long as they come from the same conan package.

        set(rmw_impl_prereqs "")
        get_filename_component(rmw_implementation_dir ${rmw_interface_lib_path} DIRECTORY)
        if(NOT EXISTS ${rmw_implementation_dir})
            message(FATAL_ERROR "Failed to extract valid parent directory from ${rmw_interface_lib_path}: got ${rmw_implementation_dir}")
        endif()
        _get_rmw_implementation_prereqs(${rmw_impl_name} ${rmw_implementation_dir} rmw_impl_prereqs)

        ##### Search RMW-specific message libraries
        # The target already has some RMW-agnostic prerequisites, i.e. message interfaces.
        # For each of them we have to add its RMW-specific counterpart.
        # Both are needed in order to use the target:
        # the formers are loaded at startup by the linker, the latters are loaded at runtime by dlopen.
        # This step is currently needed only for fastrtps middleware

        set(rmw_specific_prereqs "")
        if ("${rmw_impl_name}" STREQUAL "fastrtps")
            # This variable contains all the prerequisites of the application and the ones of the RMW implementation
            set(_prereqs ${app_prereqs} ${rmw_impl_prereqs})
            list(REMOVE_DUPLICATES _prereqs)
            foreach(prereq ${_prereqs})
                if (EXISTS ${prereq})
                    _get_rmw_specific_interfaces(${rmw_impl_name} ${prereq} specific_prereq)
                    list(APPEND rmw_specific_prereqs ${specific_prereq})
                endif()
            endforeach()
        endif()

        ##### Search interface introspection libraries
        # Some RMW implementations, such as cyclonedds, require access to the rosidl introspection
        # libraries. This usually happens when they do not define RMW-specific message libraries.
        # Introspection libraries could also be needed by some applications, regardless of the chosen
        # RMW. So we have the FORCE_INSTALL_INTROSPECTION flag to support that.

        set(introspection_prereqs "")
        foreach(prereq ${rmw_impl_prereqs})
            if (prereq MATCHES "typesupport_introspection")
                set(_requires_introspection 1)
                break()
            endif()
        endforeach()
        if (_requires_introspection OR FORCE_INSTALL_INTROSPECTION)
            set(_prereqs ${app_prereqs} ${rmw_impl_prereqs})
            list(REMOVE_DUPLICATES _prereqs)
            foreach(prereq ${_prereqs})
                _get_introspection_libraries(${rmw_impl_name} ${prereq} intros_prereq)
                list(APPEND introspection_prereqs ${intros_prereq})
            endforeach()
        endif()

        # Add to the list all the additional prerequisites that we found for this RMW implementation
        list(APPEND ros2_runtime_prereqs ${rmw_impl_prereqs} ${rmw_specific_prereqs} ${introspection_prereqs})

    endforeach()

endif()

if (ros2_runtime_prereqs)
    list(REMOVE_DUPLICATES ros2_runtime_prereqs)
    install_prerequisites(PREREQS ${ros2_runtime_prereqs} DESTINATION ${DESTINATION} ${noInstallRpath})
else()
    message(STATUS "InstallRos2 - no dependencies found for ${TARGET_PATH}.")
endif()
