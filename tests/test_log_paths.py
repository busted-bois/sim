import unittest
from pathlib import Path
from unittest.mock import patch

from src.log_paths import TIMESTAMP_TOKEN, csv_timestamp_token, resolve_log_csv_path


class LogPathsTests(unittest.TestCase):
    def test_csv_timestamp_token_format(self) -> None:
        with patch("src.log_paths.datetime") as mock_dt:
            mock_dt.now.return_value.strftime.return_value = "2026-05-23_14-30-00"
            self.assertEqual(csv_timestamp_token(), "2026-05-23_14-30-00")
            mock_dt.now.return_value.strftime.assert_called_once_with("%Y-%m-%d_%H-%M-%S")

    def test_resolve_expands_timestamp_token(self) -> None:
        root = Path("/proj")
        with patch("src.log_paths.csv_timestamp_token", return_value="2026-05-23_14-30-00"):
            path = resolve_log_csv_path(
                f"logs/trace_{TIMESTAMP_TOKEN}.csv",
                root,
                default="logs/trace.csv",
            )
        self.assertEqual(path, root / "logs/trace_2026-05-23_14-30-00.csv")

    def test_resolve_unchanged_without_token(self) -> None:
        root = Path("/proj")
        path = resolve_log_csv_path("logs/fixed.csv", root, default="logs/fixed.csv")
        self.assertEqual(path, root / "logs/fixed.csv")


if __name__ == "__main__":
    unittest.main()
