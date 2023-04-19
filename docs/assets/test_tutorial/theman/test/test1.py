import unittest
import pytest

@pytest.mark.rostest
class TestClass(unittest.TestCase):

    def test_add(self):

        self.assertEqual(2, 1)