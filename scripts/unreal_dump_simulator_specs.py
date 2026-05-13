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


def _collect_gate_specs(tokens):
    specs = []
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
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


def _collect_drone_spec(pawn_asset_path):
    pawn_asset = unreal.EditorAssetLibrary.load_asset(pawn_asset_path)
    if pawn_asset is None:
        return None

    spawned = unreal.EditorLevelLibrary.spawn_actor_from_object(
        pawn_asset,
        unreal.Vector(0.0, 0.0, 300.0),
    )
    try:
        _, extent = spawned.get_actor_bounds(True)
        camera_specs = []
        for component in spawned.get_components_by_class(unreal.ChildActorComponent):
            name = component.get_name()
            if "camera" not in name.lower():
                continue
            location = component.get_editor_property("relative_location")
            rotation = component.get_editor_property("relative_rotation")
            camera_specs.append(
                {
                    "name": name,
                    "relative_location_m": _vector_to_m(location),
                    "relative_rotation_deg": _rotator_to_deg(rotation),
                }
            )
        return {
            "asset_path": pawn_asset_path,
            "dimensions_m": _bounds_to_dimensions_m(extent),
            "cameras": camera_specs,
        }
    finally:
        unreal.EditorLevelLibrary.destroy_actor(spawned)


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


def main():
    output_path = os.environ["CODEX_SIM_SPEC_OUT"]
    map_asset = os.environ.get("CODEX_SIM_SPEC_MAP", "/Game/FlyingCPP/Maps/FlyingExampleMapV2")
    pawn_asset = os.environ.get("CODEX_SIM_SPEC_PAWN_ASSET", "/AirSim/Blueprints/BP_FlyingPawn")
    gate_tokens = [
        token.strip().lower()
        for token in os.environ.get("CODEX_SIM_SPEC_GATE_TOKENS", "gate,ring,torus").split(",")
        if token.strip()
    ]

    unreal.EditorLoadingAndSavingUtils.load_map(map_asset)
    drone_spec = _collect_drone_spec(pawn_asset)
    gates = _collect_gate_specs(gate_tokens)

    payload = {
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
