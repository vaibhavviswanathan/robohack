import cv2
import time
import os

def test_cameras():
    # Indices from xvla_client_v2.py
    HEAD_CAM_IDX = 0
    WRIST_CAM_IDX = 6
    
    indices = {'head': HEAD_CAM_IDX, 'wrist': WRIST_CAM_IDX}
    
    for name, idx in indices.items():
        print(f"Testing {name} camera (index {idx})...")
        cap = cv2.VideoCapture(idx)
        
        if not cap.isOpened():
            print(f"❌ Failed to open {name} camera")
            continue
            
        # Try to set MJPG
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Warmup
        for i in range(5):
            ret, frame = cap.read()
            if not ret:
                print(f"⚠️ Frame {i} read failed")
            else:
                print(f"✅ Frame {i} read ok: {frame.shape}")
                
        # Save one
        ret, frame = cap.read()
        if ret:
            fname = f"test_{name}.jpg"
            cv2.imwrite(fname, frame)
            size = os.path.getsize(fname)
            print(f"📸 Saved {fname} ({size} bytes)")
            
            # Check for stereo
            h, w = frame.shape[:2]
            if w > h:
                left = frame[:, :w//2]
                right = frame[:, w//2:]
                cv2.imwrite(f"test_{name}_left.jpg", left)
                cv2.imwrite(f"test_{name}_right.jpg", right)
                print(f"   Saved split images: Left ({os.path.getsize(f'test_{name}_left.jpg')}), Right ({os.path.getsize(f'test_{name}_right.jpg')})")

        
        cap.release()

if __name__ == "__main__":
    test_cameras()
