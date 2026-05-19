import unittest

from src.check_merge_markers import find_merge_conflict_markers


class MergeMarkerTests(unittest.TestCase):
    def test_repo_has_no_merge_conflict_markers(self) -> None:
        hits = find_merge_conflict_markers()
        self.assertFalse(hits, msg="\n".join(hits))


if __name__ == "__main__":
    unittest.main()
