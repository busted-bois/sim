import io
import os
import subprocess
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import Mock, patch

from src import sim_launch


class SimLaunchWslTests(unittest.TestCase):
    def test_wsl_command_uses_default_distro_when_env_unset(self) -> None:
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(
                sim_launch._wsl_command("bash", "-lc", "true"),
                ["wsl", "-e", "bash", "-lc", "true"],
            )

    def test_wsl_command_honors_aigp_wsl_distro(self) -> None:
        with patch.dict(os.environ, {"AIGP_WSL_DISTRO": "Ubuntu-22.04"}, clear=True):
            self.assertEqual(
                sim_launch._wsl_command("bash", "-lc", "true"),
                ["wsl", "-d", "Ubuntu-22.04", "-e", "bash", "-lc", "true"],
            )

    def test_manual_px4_instruction_is_powershell_safe(self) -> None:
        with patch.dict(os.environ, {"AIGP_WSL_DISTRO": "Ubuntu"}, clear=True):
            output = io.StringIO()
            with redirect_stdout(output):
                sim_launch._print_px4_bringup_instructions("127.0.0.1")

        text = output.getvalue()
        self.assertIn('wsl -d "Ubuntu" -e bash -lc "', text)
        self.assertIn(
            "cd ~/PX4-Autopilot && PX4_SIM_HOST_ADDR=127.0.0.1 make px4_sitl none_iris",
            text,
        )
        self.assertIn("AIGP_WSL_DISTRO", text)

    def test_start_px4_probe_warms_wsl_and_uses_configurable_timeout(self) -> None:
        run_results = [
            subprocess.CompletedProcess(["wsl"], 0),
            subprocess.CompletedProcess(
                ["wsl"], 0, stdout="/mnt/c/repo/scripts/launch_px4_wsl.sh\n"
            ),
        ]
        popen_mock = Mock()
        with (
            patch.dict(
                os.environ,
                {
                    "AIGP_WSL_DISTRO": "Ubuntu",
                    "AIGP_WSL_STARTUP_TIMEOUT_SECONDS": "12",
                    "AIGP_WSL_PATH_TIMEOUT_SECONDS": "34",
                },
                clear=True,
            ),
            patch.object(sim_launch.sys, "platform", "win32"),
            patch.object(Path, "is_file", return_value=True),
            patch.object(subprocess, "run", side_effect=run_results) as run_mock,
            patch.object(subprocess, "Popen", return_value=popen_mock) as popen,
        ):
            proc = sim_launch._start_px4_sitl_for_probe()

        self.assertIs(proc, popen_mock)
        self.assertEqual(run_mock.call_args_list[0].kwargs["timeout"], 12.0)
        self.assertEqual(run_mock.call_args_list[1].kwargs["timeout"], 34.0)
        self.assertEqual(
            run_mock.call_args_list[1].args[0][:4],
            ["wsl", "-d", "Ubuntu", "-e"],
        )
        self.assertEqual(
            popen.call_args.args[0],
            ["wsl", "-d", "Ubuntu", "-e", "bash", "/mnt/c/repo/scripts/launch_px4_wsl.sh"],
        )


if __name__ == "__main__":
    unittest.main()
