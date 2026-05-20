import json
import os
import statistics

import unreal


def _round_list(values):
    return [round(float(value), 6) for value in values]


def _vector_to_m(vector):
    return _round_list((vector.x / 100.0, vector.y / 100.0, vector.z / 100.0))


def _rotator_to_deg(rotator):
    return {
        "pitch": round(float(rotator.pitch), 6),
        "roll": round(float(rotator.roll), 6),
        "yaw": round(float(rotator.yaw), 6),
    }


def _bounds_to_dimensions_m(extent):
    return _round_list((extent.x * 2.0 / 100.0, extent.y * 2.0 / 100.0, extent.z * 2.0 / 100.0))


def _asset_registry():
    return unreal.AssetRegistryHelpers.get_asset_registry()


def _editor_actor_subsystem():
    try:
        return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    except Exception:
        return None


def _spawn_actor_from_class(actor_class, location):
    subsystem = _editor_actor_subsystem()
    if subsystem is not None:
        spawn = getattr(subsystem, "spawn_actor_from_class", None)
        if callable(spawn):
            try:
                return spawn(actor_class, location)
            except TypeError:
                return spawn(actor_class, location, unreal.Rotator(0.0, 0.0, 0.0))
    return unreal.EditorLevelLibrary.spawn_actor_from_class(actor_class, location)


def _spawn_actor_from_object(asset, location):
    subsystem = _editor_actor_subsystem()
    if subsystem is not None:
        spawn = getattr(subsystem, "spawn_actor_from_object", None)
        if callable(spawn):
            try:
                return spawn(asset, location)
            except TypeError:
                return spawn(asset, location, unreal.Rotator(0.0, 0.0, 0.0))
    return unreal.EditorLevelLibrary.spawn_actor_from_object(asset, location)


def _destroy_actor(actor):
    subsystem = _editor_actor_subsystem()
    if subsystem is not None:
        destroy = getattr(subsystem, "destroy_actor", None)
        if callable(destroy):
            return destroy(actor)
    return unreal.EditorLevelLibrary.destroy_actor(actor)


def _get_all_level_actors():
    subsystem = _editor_actor_subsystem()
    if subsystem is not None:
        get_all = getattr(subsystem, "get_all_level_actors", None)
        if callable(get_all):
            return list(get_all())
    return list(unreal.EditorLevelLibrary.get_all_level_actors())


def _normalized_asset_candidates(raw_path):
    cleaned = (raw_path or "").strip()
    if not cleaned:
        return []
    if cleaned.startswith("Class'") and cleaned.endswith("'"):
        cleaned = cleaned[len("Class'") : -1]
    cleaned = cleaned.strip().strip("'").strip('"')
    if cleaned.endswith("_C"):
        cleaned = cleaned[:-2]
    if "." in cleaned:
        package_path, object_name = cleaned.split(".", 1)
    else:
        package_path = cleaned
        object_name = package_path.rsplit("/", 1)[-1]
    return [
        cleaned,
        package_path,
        f"{package_path}.{object_name}",
    ]


def _normalized_class_candidates(raw_path):
    cleaned = (raw_path or "").strip()
    if not cleaned:
        return []
    if cleaned.startswith("Class'") and cleaned.endswith("'"):
        cleaned = cleaned[len("Class'") : -1]
    cleaned = cleaned.strip().strip("'").strip('"')
    if "." in cleaned:
        package_path, object_name = cleaned.split(".", 1)
    else:
        package_path = cleaned
        object_name = package_path.rsplit("/", 1)[-1]
    if not object_name.endswith("_C"):
        object_name = f"{object_name.removesuffix('_C')}_C"
    class_object_path = f"{package_path}.{object_name}"
    return [
        class_object_path,
        f"Class'{class_object_path}'",
    ]


def _load_asset_any(raw_path):
    for candidate in _normalized_asset_candidates(raw_path):
        asset = unreal.EditorAssetLibrary.load_asset(candidate)
        if asset is not None:
            return asset, candidate
    return None, None


def _load_class_any(raw_path):
    for candidate in _normalized_class_candidates(raw_path):
        pawn_class = unreal.load_class(None, candidate)
        if pawn_class is not None:
            return pawn_class, candidate
    return None, None


def _asset_data_object_path(asset_data):
    package_name = str(asset_data.package_name)
    asset_name = str(asset_data.asset_name)
    if package_name and asset_name:
        return f"{package_name}.{asset_name}"
    return package_name


def _asset_data_class_name(asset_data):
    class_path = getattr(asset_data, "asset_class_path", None)
    if class_path is not None:
        text = str(class_path)
        if "." in text:
            return text.rsplit(".", 1)[-1]
        if text:
            return text
    return str(getattr(asset_data, "asset_class", ""))


def _find_asset_by_name(asset_name, search_paths):
    if not asset_name:
        return None, None
    asset_name = asset_name.lower()
    registry = _asset_registry()
    for search_path in search_paths:
        for asset_data in registry.get_assets_by_path(search_path, recursive=True):
            if str(asset_data.asset_name).lower() != asset_name:
                continue
            asset = asset_data.get_asset()
            if asset is not None:
                return asset, _asset_data_object_path(asset_data)
    return None, None


def _resolve_pawn_asset(pawn_asset_path):
    raw_name = (pawn_asset_path or "").strip()
    if raw_name.startswith("Class'") and raw_name.endswith("'"):
        raw_name = raw_name[len("Class'") : -1]
    raw_name = raw_name.rsplit("/", 1)[-1]
    raw_name = raw_name.split(".", 1)[0]
    raw_name = raw_name.removesuffix("_C")
    asset, resolved_path = _find_asset_by_name(
        raw_name, ["/Game/FlyingCPP/Blueprints", "/Game"]
    )
    if asset is not None:
        return asset, resolved_path
    return None, None


def _spawn_pawn_actor(pawn_asset_path, pawn_asset, resolved_asset_path):
    class_candidates = [pawn_asset_path, resolved_asset_path]
    for class_path in class_candidates:
        pawn_class, resolved_class_path = _load_class_any(class_path)
        if pawn_class is None:
            continue
        spawned = _spawn_actor_from_class(pawn_class, unreal.Vector(0.0, 0.0, 300.0))
        if spawned is not None:
            return spawned, resolved_class_path

    if pawn_asset is not None:
        spawned = _spawn_actor_from_object(pawn_asset, unreal.Vector(0.0, 0.0, 300.0))
        if spawned is not None:
            return spawned, resolved_asset_path
    return None, None


def _candidate_gate(actor, tokens):
    label = actor.get_actor_label().lower()
    class_name = actor.get_class().get_name().lower()
    if any(token in label or token in class_name for token in tokens):
        return True
    for component in actor.get_components_by_class(unreal.StaticMeshComponent):
        mesh = component.get_editor_property("static_mesh")
        if mesh is None:
            continue
        mesh_name = mesh.get_name().lower()
        mesh_path = mesh.get_path_name().lower()
        if any(token in mesh_name or token in mesh_path for token in tokens):
            return True
    return False


def _collect_scene_components(actor):
    for component in actor.get_components_by_class(unreal.SceneComponent):
        try:
            location = component.get_editor_property("relative_location")
            rotation = component.get_editor_property("relative_rotation")
        except Exception:
            continue
        yield component, location, rotation


def _collect_camera_specs(actor):
    camera_specs = []
    seen = set()
    for component, location, rotation in _collect_scene_components(actor):
        name = component.get_name()
        class_name = component.get_class().get_name()
        key = (name, class_name)
        if key in seen:
            continue
        seen.add(key)
        haystack = f"{name} {class_name}".lower()
        if "camera" not in haystack:
            continue
        camera_specs.append(
            {
                "name": name,
                "class_name": class_name,
                "relative_location_m": _vector_to_m(location),
                "relative_rotation_deg": _rotator_to_deg(rotation),
            }
        )
    return camera_specs


def _collect_gate_specs(tokens):
    specs = []
    for actor in _get_all_level_actors():
        if not _candidate_gate(actor, tokens):
            continue
        _, extent = actor.get_actor_bounds(False)
        specs.append(
            {
                "label": actor.get_actor_label(),
                "class_name": actor.get_class().get_name(),
                "dimensions_m": _bounds_to_dimensions_m(extent),
            }
        )
    return specs


def _static_mesh_dimensions_m(mesh_asset):
    actor = _spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0.0, 0.0, 300.0))
    try:
        component = actor.get_component_by_class(unreal.StaticMeshComponent)
        if component is None:
            return None
        component.set_editor_property("static_mesh", mesh_asset)
        _, extent = actor.get_actor_bounds(True)
        return _bounds_to_dimensions_m(extent)
    finally:
        _destroy_actor(actor)


def _collect_gate_asset_specs(tokens):
    specs = []
    registry = _asset_registry()
    for search_path in ("/Game/FlyingCPP/Maps/_GENERATED", "/Game"):
        for asset_data in registry.get_assets_by_path(search_path, recursive=True):
            asset_name = str(asset_data.asset_name)
            object_path = _asset_data_object_path(asset_data)
            class_name = _asset_data_class_name(asset_data)
            haystack = f"{asset_name} {object_path} {class_name}".lower()
            if not any(token in haystack for token in tokens):
                continue
            asset = asset_data.get_asset()
            if asset is None or not isinstance(asset, unreal.StaticMesh):
                continue
            dimensions_m = _static_mesh_dimensions_m(asset)
            if dimensions_m is None:
                continue
            specs.append(
                {
                    "label": asset_name,
                    "class_name": asset.get_class().get_name(),
                    "asset_path": object_path,
                    "dimensions_m": dimensions_m,
                }
            )
        if specs:
            return specs
    return specs


def _collect_drone_spec(pawn_asset_path):
    pawn_asset, resolved_asset_path = _resolve_pawn_asset(pawn_asset_path)
    spawned, resolved_path = _spawn_pawn_actor(pawn_asset_path, pawn_asset, resolved_asset_path)
    if spawned is None:
        return None
    try:
        _, extent = spawned.get_actor_bounds(True)
        return {
            "asset_path": resolved_path,
            "dimensions_m": _bounds_to_dimensions_m(extent),
            "cameras": _collect_camera_specs(spawned),
        }
    finally:
        _destroy_actor(spawned)


def _physics_spec():
    settings = unreal.get_default_object(unreal.PhysicsSettings)
    return {
        "substepping": bool(settings.get_editor_property("substepping")),
        "max_substep_delta_time_s": round(
            float(settings.get_editor_property("max_substep_delta_time")),
            9,
        ),
        "max_substeps": int(settings.get_editor_property("max_substeps")),
        "tick_physics_async": bool(settings.get_editor_property("tick_physics_async")),
        "async_fixed_timestep_s": round(
            float(settings.get_editor_property("async_fixed_time_step_size")),
            9,
        ),
    }


def _gate_reference(gates):
    if not gates:
        return None
    xs = [gate["dimensions_m"][0] for gate in gates]
    ys = [gate["dimensions_m"][1] for gate in gates]
    zs = [gate["dimensions_m"][2] for gate in gates]
    return {
        "count": len(gates),
        "dimensions_m": _round_list(
            (
                statistics.median(xs),
                statistics.median(ys),
                statistics.median(zs),
            )
        ),
    }


def _runtime_camera_spec():
    raw = os.environ.get("CODEX_SIM_SPEC_CAMERA_RUNTIME", "").strip()
    if not raw:
        return None
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return None


def _metadata():
    return {
        "extracted_at_utc": os.environ.get("CODEX_SIM_SPEC_EXTRACTED_AT_UTC", ""),
        "project_path": os.environ.get("CODEX_SIM_SPEC_PROJECT_PATH", ""),
        "config_sha256": os.environ.get("CODEX_SIM_SPEC_CONFIG_SHA256", ""),
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "runtime_camera_spec": _runtime_camera_spec(),
    }


def main():
    output_path = os.environ["CODEX_SIM_SPEC_OUT"]
    map_asset = os.environ.get("CODEX_SIM_SPEC_MAP", "/Game/FlyingCPP/Maps/FlyingExampleMapV2")
    pawn_asset = os.environ.get(
        "CODEX_SIM_SPEC_PAWN_ASSET",
        "/Game/FlyingCPP/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C",
    )
    gate_tokens = [
        token.strip().lower()
        for token in os.environ.get("CODEX_SIM_SPEC_GATE_TOKENS", "gate,ring,torus").split(",")
        if token.strip()
    ]

    unreal.EditorLoadingAndSavingUtils.load_map(map_asset)
    drone_spec = _collect_drone_spec(pawn_asset)
    gates = _collect_gate_specs(gate_tokens)
    if not gates:
        gates = _collect_gate_asset_specs(gate_tokens)

    payload = {
        "metadata": _metadata(),
        "map_asset": map_asset,
        "physics": _physics_spec(),
        "drone": drone_spec,
        "gates": gates,
        "gate_reference": _gate_reference(gates),
    }

    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)

    unreal.log(f"Codex simulator specification snapshot written to {output_path}")
    unreal.SystemLibrary.quit_editor()


if __name__ == "__main__":
    main()
