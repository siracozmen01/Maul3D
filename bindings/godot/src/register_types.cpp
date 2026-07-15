// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "m3_world_3d.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

static void InitializeMaul3DModule(ModuleInitializationLevel level)
{
    if (level != MODULE_INITIALIZATION_LEVEL_SCENE)
    {
        return;
    }
    ClassDB::register_class<maul3d_godot::M3World3D>();
}

static void UninitializeMaul3DModule(ModuleInitializationLevel level)
{
    (void)level;
}

extern "C"
{
    GDExtensionBool GDE_EXPORT maul3d_library_init(GDExtensionInterfaceGetProcAddress get_proc,
                                                   const GDExtensionClassLibraryPtr library,
                                                   GDExtensionInitialization* initialization)
    {
        godot::GDExtensionBinding::InitObject init_obj(get_proc, library, initialization);
        init_obj.register_initializer(InitializeMaul3DModule);
        init_obj.register_terminator(UninitializeMaul3DModule);
        init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
        return init_obj.init();
    }
}
