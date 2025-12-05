
import numpy as np
import math
import unittest

def _wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi

class TestPathContinuity(unittest.TestCase):
    def test_circular_path_continuity(self):
        # Generate a perfect circle
        R = 10.0
        num_points = 100
        thetas = np.linspace(0, 2*np.pi, num_points, endpoint=False) # Open loop points
        
        # Create path points
        x = R * np.cos(thetas)
        y = R * np.sin(thetas)
        
        # Simulate the logic in _path_cb (current implementation logic approx)
        # The current implementation calculates curvature using 3 points.
        # It doesn't wrap around.
        
        data = np.zeros((num_points, 4))
        data[:, 0] = x
        data[:, 1] = y
        
        # --- Current Logic Simulation (Flawed) ---
        # Curvature
        p_m = data[:-2, :2]
        p = data[1:-1, :2]
        p_p = data[2:, :2]

        a = np.linalg.norm(p - p_m, axis=1)
        b = np.linalg.norm(p_p - p, axis=1)
        c = np.linalg.norm(p_p - p_m, axis=1)

        s_tri = (a + b + c) / 2
        area = np.sqrt(np.abs(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c)))
        kappa = 4 * area / (a * b * c + 1e-6)
        
        # This leaves start and end with 0 curvature or copied curvature
        # In current code:
        # data[1:-1, 3] = kappa
        # data[0, 3] = kappa[0]
        # data[-1, 3] = kappa[-1]
        
        # Check discontinuity
        # Ideally kappa should be 1/R = 0.1 everywhere
        
        # Let's implement the PROPOSED fix logic and verify it gives better results
        
        # --- Proposed Logic ---
        # Check if closed (it is effectively closed if we treat it as such)
        is_closed = True 
        
        if is_closed:
            # Pad
            pad_len = 5
            x_pad_pre = x[-pad_len:]
            y_pad_pre = y[-pad_len:]
            x_pad_post = x[:pad_len]
            y_pad_post = y[:pad_len]
            
            x_aug = np.concatenate([x_pad_pre, x, x_pad_post])
            y_aug = np.concatenate([y_pad_pre, y, y_pad_post])
            
            points_aug = np.column_stack([x_aug, y_aug])
            
            # Calc curvature on augmented
            p_m_aug = points_aug[:-2]
            p_aug = points_aug[1:-1]
            p_p_aug = points_aug[2:]
            
            a = np.linalg.norm(p_aug - p_m_aug, axis=1)
            b = np.linalg.norm(p_p_aug - p_aug, axis=1)
            c = np.linalg.norm(p_p_aug - p_m_aug, axis=1)
            
            s_tri = (a + b + c) / 2
            area = np.sqrt(np.abs(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c)))
            kappa_aug = 4 * area / (a * b * c + 1e-6)
            
            # Extract valid region
            # We padded `pad_len` at start, so the first real point is at index `pad_len` in `points_aug`
            # `kappa_aug` corresponds to points `p_aug`, which starts at index 1 of `points_aug`.
            # So `kappa_aug[0]` corresponds to `points_aug[1]` (which is `x_pad_pre[-4]`)
            # We want curvature for `x[0]`. `x[0]` is at `points_aug[pad_len]`.
            # `points_aug[pad_len]` corresponds to `kappa_aug[pad_len - 1]`.
            
            kappa_valid = kappa_aug[pad_len-1 : pad_len-1 + num_points]
            
            # Verify
            expected_k = 1.0/R
            max_error = np.max(np.abs(kappa_valid - expected_k))
            print(f"Max curvature error with fix: {max_error}")
            
            # Check continuity at wrap
            k_start = kappa_valid[0]
            k_end = kappa_valid[-1]
            print(f"Start k: {k_start}, End k: {k_end}")
            
            self.assertTrue(np.isclose(k_start, k_end, atol=1e-3), "Curvature should be continuous at wrap")
            self.assertTrue(max_error < 1e-2, f"Curvature calculation should be accurate. Max error: {max_error}")

if __name__ == '__main__':
    unittest.main()
